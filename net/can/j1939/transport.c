// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2010-2011 EIA Electronics, Kurt Van Dijck <kurt.van.dijck@eia.be>
// Copyright (c) 2017-2018 Pengutronix, Marc Kleine-Budde <kernel@pengutronix.de>
// Copyright (c) 2017-2018 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
// Copyright (c) 2018 Protonic, Robin van der Gracht <robin@protonic.nl>

#include <linux/can/skb.h>

#include "j1939-priv.h"

#define J1939_REGULAR 0
#define J1939_EXTENDED 1

#define J1939_ETP_PGN_CTL 0xc800
#define J1939_ETP_PGN_DAT 0xc700
#define J1939_TP_PGN_CTL 0xec00
#define J1939_TP_PGN_DAT 0xeb00

#define J1939_TP_CMD_RTS 0x10
#define J1939_TP_CMD_CTS 0x11
#define J1939_TP_CMD_EOMA 0x13
#define J1939_TP_CMD_BAM 0x20
#define J1939_TP_CMD_ABORT 0xff

#define J1939_ETP_CMD_RTS 0x14
#define J1939_ETP_CMD_CTS 0x15
#define J1939_ETP_CMD_DPO 0x16
#define J1939_ETP_CMD_EOMA 0x17
#define J1939_ETP_CMD_ABORT 0xff

enum j1939_xtp_abort {
	J1939_XTP_ABORT_NO_ERROR = 0,
	J1939_XTP_ABORT_BUSY = 1,
	J1939_XTP_ABORT_RESOURCE = 2,
	J1939_XTP_ABORT_TIMEOUT = 3,
	J1939_XTP_ABORT_GENERIC = 4,
	J1939_XTP_ABORT_FAULT = 5,
};

#define J1939_MAX_TP_PACKET_SIZE (7 * 0xff)
#define J1939_MAX_ETP_PACKET_SIZE (7 * 0x00ffffff)

static unsigned int j1939_tp_block = 255;
static unsigned int j1939_tp_max_packet_size = J1939_MAX_ETP_PACKET_SIZE;
static unsigned int j1939_tp_retry_ms = 20;
static unsigned int j1939_tp_packet_delay;
static unsigned int j1939_tp_padding = 1;

struct j1939_session {
	struct j1939_priv *priv;
	struct list_head list;
	struct kref kref;
	spinlock_t lock;

	/* ifindex, src, dst, pgn define the session block
	 * the are _never_ modified after insertion in the list
	 * this decreases locking problems a _lot_
	 */
	struct j1939_sk_buff_cb *skcb;
	struct sk_buff *skb;

	/* all tx related stuff (last_txcmd, pkt.tx)
	 * is protected (modified only) with the txtimer hrtimer
	 * 'total' & 'block' are never changed,
	 * last_cmd, last & block are protected by ->lock
	 * this means that the tx may run after cts is received that should
	 * have stopped tx, but this time discrepancy is never avoided anyhow
	 */
	u8 last_cmd, last_txcmd;
	bool transmission;
	bool extd;
	struct {
		/* these do not require 16 bit, they should fit in u8
		 * but putting in int makes it easier to deal with
		 */
		unsigned int total, done, last, tx;
		unsigned int block; /* for TP */
		unsigned int dpo; /* for ETP */
	} pkt;
	struct hrtimer txtimer, rxtimer;
};

/* helpers */
static inline void j1939_fix_cb(struct j1939_sk_buff_cb *skcb)
{
	skcb->msg_flags &= ~MSG_SYN;
}

static inline struct list_head *j1939_sessionq(struct j1939_priv *priv,
					       bool extd)
{
	if (extd)
		return &priv->tp_extsessionq;
	else
		return &priv->tp_sessionq;
}

static inline void j1939_session_list_lock(struct j1939_priv *priv)
{
	spin_lock_bh(&priv->tp_session_list_lock);
}

static inline void j1939_session_list_unlock(struct j1939_priv *priv)
{
	spin_unlock_bh(&priv->tp_session_list_lock);
}

static void j1939_session_list_add(struct j1939_session *session,
				   struct list_head *list)
{
	list_add_tail(&session->list, list);
}

static void j1939_session_list_del(struct j1939_session *session)
{
	list_del_init(&session->list);
}

static inline void j1939_session_get(struct j1939_session *session)
{
	kref_get(&session->kref);
}

static void j1939_session_destroy(struct j1939_session *session)
{
	j1939_session_list_lock(session->priv);
	j1939_session_list_del(session);
	j1939_session_list_unlock(session->priv);
	kfree_skb(session->skb);
	j1939_priv_put(session->priv);
	kfree(session);
}

static void __j1939_session_release(struct kref *kref)
{
	struct j1939_session *session = container_of(kref, struct j1939_session,
						     kref);

	j1939_session_destroy(session);
}

static inline void j1939_session_put(struct j1939_session *session)
{
	kref_put(&session->kref, __j1939_session_release);
}

static void j1939_session_txtimer_cancel(struct j1939_session *session)
{
	if (hrtimer_cancel(&session->txtimer))
		j1939_session_put(session);
}

static void j1939_session_rxtimer_cancel(struct j1939_session *session)
{
	if (hrtimer_cancel(&session->rxtimer))
		j1939_session_put(session);
}

static void j1939_session_timers_cancel(struct j1939_session *session)
{
	j1939_session_txtimer_cancel(session);
	j1939_session_rxtimer_cancel(session);
}

static inline bool j1939_cb_is_broadcast(const struct j1939_sk_buff_cb *skcb)
{
	return (!skcb->addr.dst_name && (skcb->addr.da == 0xff));
}

/* transport status locking */
static inline void j1939_session_lock(struct j1939_session *session)
{
	spin_lock_bh(&session->lock);
}

static inline void j1939_session_unlock(struct j1939_session *session)
{
	spin_unlock_bh(&session->lock);
}

/* see if we are receiver
 * returns 0 for broadcasts, although we will receive them
 */
static inline int j1939_tp_im_receiver(struct sk_buff *skb)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);

	return skcb->dst_flags & J1939_ECU_LOCAL;
}

/* see if we are sender */
static inline int j1939_tp_im_transmitter(struct sk_buff *skb)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);

	return skcb->src_flags & J1939_ECU_LOCAL;
}

/* see if we are involved as either receiver or transmitter */
static int j1939_tp_im_involved(struct sk_buff *skb, bool swap)
{
	if (swap)
		return j1939_tp_im_receiver(skb);
	else
		return j1939_tp_im_transmitter(skb);
}

static int j1939_tp_im_involved_anydir(struct sk_buff *skb)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);

	return (skcb->src_flags | skcb->dst_flags) & J1939_ECU_LOCAL;
}

/* extract pgn from flow-ctl message */
static inline pgn_t j1939_xtp_ctl_to_pgn(const u8 *dat)
{
	pgn_t pgn;

	pgn = (dat[7] << 16) | (dat[6] << 8) | (dat[5] << 0);
	if (j1939_pgn_is_pdu1(pgn))
		pgn &= 0xffff00;
	return pgn;
}

static inline unsigned int j1939_tp_ctl_to_size(const u8 *dat)
{
	return (dat[2] << 8) + (dat[1] << 0);
}

static inline unsigned int j1939_etp_ctl_to_packet(const u8 *dat)
{
	return (dat[4] << 16) | (dat[3] << 8) | (dat[2] << 0);
}

static inline unsigned int j1939_etp_ctl_to_size(const u8 *dat)
{
	return (dat[4] << 24) | (dat[3] << 16) |
		(dat[2] << 8) | (dat[1] << 0);
}

/* find existing session:
 * reverse: swap cb's src & dst
 * there is no problem with matching broadcasts, since
 * broadcasts (no dst, no da) would never call this
 * with reverse == true
 */
static bool j1939_session_match(struct j1939_session *session,
				struct sk_buff *skb, bool reverse)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);

	if (can_skb_prv(session->skb)->ifindex != can_skb_prv(skb)->ifindex)
		return false;
	if (reverse) {
		if (session->skcb->addr.src_name) {
			if (session->skcb->addr.src_name != skcb->addr.dst_name)
				return false;
		} else if (session->skcb->addr.sa != skcb->addr.da) {
			return false;
		}

		if (session->skcb->addr.dst_name) {
			if (session->skcb->addr.dst_name != skcb->addr.src_name)
				return false;
		} else if (session->skcb->addr.da != skcb->addr.sa) {
			return false;
		}
	} else {
		if (session->skcb->addr.src_name) {
			if (session->skcb->addr.src_name != skcb->addr.src_name)
				return false;
		} else if (session->skcb->addr.sa != skcb->addr.sa) {
			return false;
		}

		if (session->skcb->addr.dst_name) {
			if (session->skcb->addr.dst_name != skcb->addr.dst_name)
				return false;
		} else if (session->skcb->addr.da != skcb->addr.da) {
			return false;
		}
	}

	return true;
}

static struct
j1939_session *j1939_session_get_by_skb_locked(struct j1939_priv *priv,
					       struct list_head *root,
					       struct sk_buff *skb,
					       bool reverse)
{
	struct j1939_session *session;

	lockdep_assert_held(&priv->tp_session_list_lock);

	list_for_each_entry(session, root, list) {
		j1939_session_get(session);
		if (j1939_session_match(session, skb, reverse))
			return session;
		j1939_session_put(session);
	}

	return NULL;
}

static struct j1939_session *j1939_session_get_by_skb(struct j1939_priv *priv,
						      struct list_head *root,
						      struct sk_buff *skb,
						      bool reverse)
{
	struct j1939_session *session;

	j1939_session_list_lock(priv);
	session = j1939_session_get_by_skb_locked(priv, root, skb, reverse);
	j1939_session_list_unlock(priv);

	return session;
}

static void j1939_skbcb_swap(struct j1939_sk_buff_cb *skcb)
{
	swap(skcb->addr.dst_name, skcb->addr.src_name);
	swap(skcb->addr.da, skcb->addr.sa);
	swap(skcb->dst_flags, skcb->src_flags);
}

static struct sk_buff *j1939_tp_tx_dat_new(struct sk_buff *related,
					   bool extd, bool ctl,
					   bool swap_src_dst)
{
	struct sk_buff *skb;
	struct j1939_sk_buff_cb *skcb;

	skb = alloc_skb(sizeof(struct can_frame) + sizeof(struct can_skb_priv),
			GFP_ATOMIC);
	if (unlikely(!skb))
		return ERR_PTR(-ENOMEM);

	skb->dev = related->dev;
	can_skb_reserve(skb);
	can_skb_prv(skb)->ifindex = can_skb_prv(related)->ifindex;
	/* reserve CAN header */
	skb_reserve(skb, offsetof(struct can_frame, data));
	skb->protocol = related->protocol;
	skb->pkt_type = related->pkt_type;
	skb->ip_summed = related->ip_summed;

	memcpy(skb->cb, related->cb, sizeof(skb->cb));
	skcb = j1939_skb_to_cb(skb);
	j1939_fix_cb(skcb);
	if (swap_src_dst)
		j1939_skbcb_swap(skcb);

	if (ctl) {
		if (extd)
			skcb->addr.pgn = J1939_ETP_PGN_CTL;
		else
			skcb->addr.pgn = J1939_TP_PGN_CTL;
	} else {
		if (extd)
			skcb->addr.pgn = J1939_ETP_PGN_DAT;
		else
			skcb->addr.pgn = J1939_TP_PGN_DAT;
	}

	return skb;
}

/* TP transmit packet functions */
static int j1939_tp_tx_dat(struct sk_buff *related, bool extd,
			   const u8 *dat, int len)
{
	struct sk_buff *skb;

	skb = j1939_tp_tx_dat_new(related, extd, false, false);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	skb_put_data(skb, dat, len);
	if (j1939_tp_padding && len < 8)
		memset(skb_put(skb, 8 - len), 0xff, 8 - len);

	return j1939_send(skb);
}

static int j1939_xtp_do_tx_ctl(struct sk_buff *related, bool extd,
			       bool swap_src_dst, pgn_t pgn, const u8 *dat)
{
	struct sk_buff *skb;
	u8 *skdat;

	if (!j1939_tp_im_involved(related, swap_src_dst))
		return 0;

	skb = j1939_tp_tx_dat_new(related, extd, true, swap_src_dst);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	skdat = skb_put(skb, 8);
	memcpy(skdat, dat, 5);
	skdat[5] = (pgn >> 0);
	skdat[6] = (pgn >> 8);
	skdat[7] = (pgn >> 16);

	return j1939_send(skb);
}

static inline int j1939_tp_tx_ctl(struct j1939_session *session,
				  bool swap_src_dst, const u8 *dat)
{
	return j1939_xtp_do_tx_ctl(session->skb, session->extd, swap_src_dst,
				   session->skcb->addr.pgn, dat);
}

static int j1939_xtp_tx_abort(struct sk_buff *related, bool extd,
			      bool swap_src_dst, enum j1939_xtp_abort err,
			      pgn_t pgn)
{
	u8 dat[5];

	if (!j1939_tp_im_involved(related, swap_src_dst))
		return 0;

	memset(dat, 0xff, sizeof(dat));
	dat[0] = J1939_TP_CMD_ABORT;
	if (extd)
		dat[1] = J1939_XTP_ABORT_GENERIC;
	else
		dat[1] = err;
	return j1939_xtp_do_tx_ctl(related, extd, swap_src_dst, pgn, dat);
}

static inline void j1939_tp_schedule_txtimer(struct j1939_session *session,
					     int msec)
{
	j1939_session_get(session);
	hrtimer_start(&session->txtimer, ms_to_ktime(msec),
		      HRTIMER_MODE_REL_SOFT);
}

static inline void j1939_tp_set_rxtimeout(struct j1939_session *session,
					  int msec)
{
	j1939_session_rxtimer_cancel(session);
	j1939_session_get(session);
	hrtimer_start(&session->rxtimer, ms_to_ktime(msec),
		      HRTIMER_MODE_REL_SOFT);
}

/* transmit function */
static int j1939_tp_txnext(struct j1939_session *session)
{
	u8 dat[8];
	const u8 *tpdat;
	int ret, offset, pkt_done, pkt_end;
	unsigned int pkt, len, pdelay;

	memset(dat, 0xff, sizeof(dat));

	switch (session->last_cmd) {
	case 0:
		if (!j1939_tp_im_transmitter(session->skb))
			break;
		dat[1] = (session->skb->len >> 0);
		dat[2] = (session->skb->len >> 8);
		dat[3] = session->pkt.total;
		if (session->extd) {
			dat[0] = J1939_ETP_CMD_RTS;
			dat[1] = (session->skb->len >> 0);
			dat[2] = (session->skb->len >> 8);
			dat[3] = (session->skb->len >> 16);
			dat[4] = (session->skb->len >> 24);
		} else if (j1939_cb_is_broadcast(session->skcb)) {
			dat[0] = J1939_TP_CMD_BAM;
			/* fake cts for broadcast */
			session->pkt.tx = 0;
		} else {
			dat[0] = J1939_TP_CMD_RTS;
			dat[4] = dat[3];
		}
		if (dat[0] == session->last_txcmd)
			/* done already */
			break;
		ret = j1939_tp_tx_ctl(session, false, dat);
		if (ret < 0)
			goto failed;
		session->last_txcmd = dat[0];
		/* must lock? */
		if (dat[0] == J1939_TP_CMD_BAM)
			j1939_tp_schedule_txtimer(session, 50);
		j1939_tp_set_rxtimeout(session, 1250);
		break;
	case J1939_TP_CMD_RTS:
	case J1939_ETP_CMD_RTS: /* fallthrough */
		if (!j1939_tp_im_receiver(session->skb))
			break;
 tx_cts:
		ret = 0;
		len = session->pkt.total - session->pkt.done;
		len = min3(len, session->pkt.block, j1939_tp_block ?: 255);

		if (session->extd) {
			pkt = session->pkt.done + 1;
			dat[0] = J1939_ETP_CMD_CTS;
			dat[1] = len;
			dat[2] = (pkt >> 0);
			dat[3] = (pkt >> 8);
			dat[4] = (pkt >> 16);
		} else {
			dat[0] = J1939_TP_CMD_CTS;
			dat[1] = len;
			dat[2] = session->pkt.done + 1;
		}
		if (dat[0] == session->last_txcmd)
			/* done already */
			break;
		ret = j1939_tp_tx_ctl(session, true, dat);
		if (ret < 0)
			goto failed;
		if (len)
			/* only mark cts done when len is set */
			session->last_txcmd = dat[0];
		j1939_tp_set_rxtimeout(session, 1250);
		break;
	case J1939_ETP_CMD_CTS:
		if (j1939_tp_im_transmitter(session->skb) &&
		    session->extd &&
		    session->last_txcmd != J1939_ETP_CMD_DPO) {
			/* do dpo */
			dat[0] = J1939_ETP_CMD_DPO;
			session->pkt.dpo = session->pkt.done;
			pkt = session->pkt.dpo;
			dat[1] = session->pkt.last - session->pkt.done;
			dat[2] = (pkt >> 0);
			dat[3] = (pkt >> 8);
			dat[4] = (pkt >> 16);
			ret = j1939_tp_tx_ctl(session, false, dat);
			if (ret < 0)
				goto failed;
			session->last_txcmd = dat[0];
			j1939_tp_set_rxtimeout(session, 1250);
			session->pkt.tx = session->pkt.done;
		}
		/* fallthrough */
	case J1939_TP_CMD_CTS: /* fallthrough */
	case 0xff: /* did some data */
	case J1939_ETP_CMD_DPO: /* fallthrough */
		if ((session->extd ||
		     !j1939_cb_is_broadcast(session->skcb)) &&
		    j1939_tp_im_receiver(session->skb)) {
			if (session->pkt.done >= session->pkt.total) {
				if (session->extd) {
					dat[0] = J1939_ETP_CMD_EOMA;
					dat[1] = session->skb->len >> 0;
					dat[2] = session->skb->len >> 8;
					dat[3] = session->skb->len >> 16;
					dat[4] = session->skb->len >> 24;
				} else {
					dat[0] = J1939_TP_CMD_EOMA;
					dat[1] = session->skb->len;
					dat[2] = session->skb->len >> 8;
					dat[3] = session->pkt.total;
				}
				if (dat[0] == session->last_txcmd)
					/* done already */
					break;
				ret = j1939_tp_tx_ctl(session, true, dat);
				if (ret < 0)
					goto failed;
				session->last_txcmd = dat[0];
				j1939_tp_set_rxtimeout(session, 1250);
				/* wait for the EOMA packet to come in */
				break;
			} else if (session->pkt.done >= session->pkt.last) {
				session->last_txcmd = 0;
				goto tx_cts;
			}
		}
	case J1939_TP_CMD_BAM: /* fallthrough */
		if (!j1939_tp_im_transmitter(session->skb))
			break;
		tpdat = session->skb->data;
		ret = 0;
		pkt_done = 0;
		if (!session->extd && j1939_cb_is_broadcast(session->skcb))
			pkt_end = session->pkt.total;
		else
			pkt_end = session->pkt.last;

		while (session->pkt.tx < pkt_end) {
			dat[0] = session->pkt.tx - session->pkt.dpo + 1;
			offset = session->pkt.tx * 7;
			len = session->skb->len - offset;
			if (len > 7)
				len = 7;
			memcpy(&dat[1], &tpdat[offset], len);
			ret = j1939_tp_tx_dat(session->skb, session->extd,
					      dat, len + 1);
			if (ret < 0)
				break;
			session->last_txcmd = 0xff;
			++pkt_done;
			++session->pkt.tx;
			pdelay = j1939_cb_is_broadcast(session->skcb) ? 50 :
				j1939_tp_packet_delay;
			if (session->pkt.tx < session->pkt.total && pdelay) {
				j1939_tp_schedule_txtimer(session, pdelay);
				break;
			}
		}
		if (pkt_done)
			j1939_tp_set_rxtimeout(session, 250);
		if (ret)
			goto failed;
		break;
	}

	return 0;

 failed:
	return ret;
}

/* timer & scheduler functions */
static enum hrtimer_restart j1939_tp_txtimer(struct hrtimer *hrtimer)
{
	struct j1939_session *session =
		container_of(hrtimer, struct j1939_session, txtimer);
	int ret;

	ret = j1939_tp_txnext(session);
	if (ret < 0)
		j1939_tp_schedule_txtimer(session, j1939_tp_retry_ms ?: 20);
	j1939_session_put(session);

	return HRTIMER_NORESTART;
}

/* session completion functions */
static void __j1939_session_drop(struct j1939_session *session)
{
	struct j1939_priv *priv = session->priv;

	if (session->transmission) {
		if (session->skb && session->skb->sk)
			j1939_sock_pending_del(session->skb->sk);
		wake_up_all(&priv->tp_wait);
	}
}

static void j1939_session_completed(struct j1939_session *session)
{
	/* distribute among j1939 receivers */
	j1939_sk_recv(session->skb);
	__j1939_session_drop(session);
}

static void j1939_session_cancel(struct j1939_session *session,
				 enum j1939_xtp_abort err)
{
	/* do not send aborts on incoming broadcasts */
	if (err && j1939_tp_im_involved_anydir(session->skb) &&
	    !j1939_cb_is_broadcast(session->skcb))
		j1939_xtp_tx_abort(session->skb, session->extd,
				   !(session->skcb->src_flags & J1939_ECU_LOCAL),
				   err, session->skcb->addr.pgn);

	__j1939_session_drop(session);
}

static enum hrtimer_restart j1939_tp_rxtimer(struct hrtimer *hrtimer)
{
	struct j1939_session *session = container_of(hrtimer,
						     struct j1939_session,
						     rxtimer);
	struct j1939_priv *priv = session->priv;

	netdev_alert(priv->ndev, "%s: timeout on %i\n", __func__,
		     can_skb_prv(session->skb)->ifindex);
	j1939_session_txtimer_cancel(session);
	j1939_session_cancel(session, J1939_XTP_ABORT_TIMEOUT);
	j1939_session_put(session);

	return HRTIMER_NORESTART;
}

/* receive packet functions */
static void j1939_xtp_rx_bad_message_one(struct j1939_priv *priv,
					 struct sk_buff *skb, bool extd,
					 bool reverse)
{
	struct j1939_session *session;
	pgn_t pgn;

	pgn = j1939_xtp_ctl_to_pgn(skb->data);
	session = j1939_session_get_by_skb(priv, j1939_sessionq(priv, extd),
					   skb, reverse);
	if (session /*&& (session->skcb->addr.pgn == pgn)*/) {
		/* do not allow TP control messages on 2 pgn's */
		j1939_session_cancel(session, J1939_XTP_ABORT_FAULT);
		goto out_session_put;
	}
	j1939_xtp_tx_abort(skb, extd, false, J1939_XTP_ABORT_FAULT, pgn);
	if (!session)
		return;

 out_session_put:
	j1939_session_put(session);
}

/* abort packets may come in 2 directions */
static void j1939_xtp_rx_bad_message(struct j1939_priv *priv,
				     struct sk_buff *skb, bool extd)
{
	netdev_info(priv->ndev, "%s, pgn %05x\n", __func__,
		    j1939_xtp_ctl_to_pgn(skb->data));

	j1939_xtp_rx_bad_message_one(priv, skb, extd, false);
	j1939_xtp_rx_bad_message_one(priv, skb, extd, true);
}

static void j1939_xtp_rx_abort_one(struct j1939_priv *priv, struct sk_buff *skb,
				   bool extd, bool reverse)
{
	struct j1939_session *session;
	pgn_t pgn;

	pgn = j1939_xtp_ctl_to_pgn(skb->data);
	session = j1939_session_get_by_skb(priv, j1939_sessionq(priv, extd),
					   skb, reverse);
	if (!session)
		return;
	if (session->transmission && !session->last_txcmd) {
		/* empty block:
		 * do not drop session when a transmit session did not
		 * start yet
		 */
	} else if (session->skcb->addr.pgn == pgn) {
		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_NO_ERROR);
	}

	/* TODO: maybe cancel current connection
	 * as another pgn was communicated
	 */
	j1939_session_put(session);
}

/* abort packets may come in 2 directions */
static void j1939_xtp_rx_abort(struct j1939_priv *priv, struct sk_buff *skb,
			       bool extd)
{
	netdev_info(priv->ndev, "%s %i, %05x\n", __func__,
		    can_skb_prv(skb)->ifindex, j1939_xtp_ctl_to_pgn(skb->data));

	j1939_xtp_rx_abort_one(priv, skb, extd, false);
	j1939_xtp_rx_abort_one(priv, skb, extd, true);
}

static void j1939_xtp_rx_eoma(struct j1939_priv *priv, struct sk_buff *skb,
			      bool extd)
{
	struct j1939_session *session;
	pgn_t pgn;

	/* end of tx cycle */
	pgn = j1939_xtp_ctl_to_pgn(skb->data);
	session = j1939_session_get_by_skb(priv, j1939_sessionq(priv, extd),
					   skb, true);
	if (!session) {
		/* strange, we had EOMA on closed connection
		 * do nothing, as EOMA closes the connection anyway
		 */
		return;
	}

	j1939_session_timers_cancel(session);
	if (session->skcb->addr.pgn != pgn) {
		j1939_xtp_tx_abort(skb, extd, true, J1939_XTP_ABORT_BUSY, pgn);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);
	} else {
		/* transmitted without problems */
		j1939_session_completed(session);
	}

	j1939_session_put(session);
}

static void j1939_xtp_rx_cts(struct j1939_priv *priv, struct sk_buff *skb,
			     bool extd)
{
	struct j1939_session *session;
	pgn_t pgn;
	unsigned int pkt;
	const u8 *dat;

	dat = skb->data;
	pgn = j1939_xtp_ctl_to_pgn(skb->data);
	session = j1939_session_get_by_skb(priv, j1939_sessionq(priv, extd),
					   skb, true);
	if (!session) {
		/* 'CTS shall be ignored' */
		return;
	}

	if (session->skcb->addr.pgn != pgn) {
		/* what to do? */
		j1939_xtp_tx_abort(skb, extd, true, J1939_XTP_ABORT_BUSY, pgn);
		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);
		goto out_session_put;
	}

	j1939_session_lock(session);
	if (extd)
		pkt = j1939_etp_ctl_to_packet(dat);
	else
		pkt = dat[2];
	if (!pkt) {
		goto out_session_unlock;
	} else if (dat[1] > session->pkt.block /* 0xff for etp */) {
		goto out_session_unlock;
	} else {
		/* set packet counters only when not CTS(0) */
		session->pkt.done = pkt - 1;
		session->pkt.last = session->pkt.done + dat[1];
		if (session->pkt.last > session->pkt.total)
			/* safety measure */
			session->pkt.last = session->pkt.total;
		/* TODO: do not set tx here, do it in txtimer */
		session->pkt.tx = session->pkt.done;
	}

	session->last_cmd = dat[0];
	j1939_session_unlock(session);
	if (dat[1]) {
		j1939_tp_set_rxtimeout(session, 1250);
		if (j1939_tp_im_transmitter(session->skb))
			j1939_tp_schedule_txtimer(session, 0);
	} else {
		/* CTS(0) */
		j1939_tp_set_rxtimeout(session, 550);
	}
	j1939_session_put(session);
	return;

 out_session_unlock:
	j1939_session_unlock(session);
	j1939_session_timers_cancel(session);
	j1939_session_cancel(session, J1939_XTP_ABORT_FAULT);
 out_session_put:
	j1939_session_put(session);
}

static struct j1939_session *j1939_session_new(struct j1939_priv *priv,
					       struct sk_buff *skb)
{
	struct j1939_session *session;

	session = kzalloc(sizeof(*session), gfp_any());
	if (!session)
		return NULL;
	INIT_LIST_HEAD(&session->list);
	spin_lock_init(&session->lock);
	kref_init(&session->kref);

	j1939_priv_get(priv);
	session->priv = priv;
	/* corresponding skb_unref() is in j1939_session_fresh_new */
	session->skb = skb_get(skb);
	session->skcb = j1939_skb_to_cb(session->skb);

	hrtimer_init(&session->txtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL_SOFT);
	session->txtimer.function = j1939_tp_txtimer;
	hrtimer_init(&session->rxtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL_SOFT);
	session->rxtimer.function = j1939_tp_rxtimer;

	return session;
}

static struct j1939_session *j1939_session_fresh_new(struct j1939_priv *priv,
						     int size,
						     struct sk_buff *rel_skb,
						     pgn_t pgn)
{
	const struct j1939_sk_buff_cb *rel_skcb = j1939_skb_to_cb(rel_skb);
	struct sk_buff *skb;
	struct j1939_sk_buff_cb *skcb;
	struct j1939_session *session;

	skb = alloc_skb(size + sizeof(struct can_skb_priv), GFP_ATOMIC);
	if (unlikely(!skb))
		return NULL;

	skb->dev = rel_skb->dev;
	can_skb_reserve(skb);
	can_skb_prv(skb)->ifindex = can_skb_prv(rel_skb)->ifindex;
	skcb = j1939_skb_to_cb(skb);
	memcpy(skcb, rel_skcb, sizeof(*skcb));
	j1939_fix_cb(skcb);
	skcb->addr.pgn = pgn;

	session = j1939_session_new(priv, skb);
	if (!session) {
		kfree_skb(skb);
		return NULL;
	}

	/* alloc data area */
	skb_put(skb, size);
	/* skb is recounted in j1939_session_new() */
	WARN_ON_ONCE(skb_unref(skb));
	return session;
}

static int j1939_session_insert(struct j1939_session *session)
{
	struct j1939_priv *priv = session->priv;
	struct j1939_session *pending;
	int ret = 0;

	pending = j1939_session_get_by_skb(priv,
					   j1939_sessionq(priv, session->extd),
					   session->skb, false);
	if (pending) {
		j1939_session_put(pending);
		ret = -EAGAIN;
	} else {
		j1939_session_list_lock(priv);
		j1939_session_list_add(session,
				       j1939_sessionq(priv, session->extd));
		j1939_session_list_unlock(priv);
	}

	return ret;
}

static void j1939_xtp_rx_rts(struct j1939_priv *priv, struct sk_buff *skb,
			     bool extd)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	struct j1939_session *session;
	int len;
	const u8 *dat;
	pgn_t pgn;

	dat = skb->data;
	pgn = j1939_xtp_ctl_to_pgn(dat);

	if (dat[0] == J1939_TP_CMD_RTS && j1939_cb_is_broadcast(skcb)) {
		netdev_alert(priv->ndev, "%s: rts without destination (%i %02x)\n",
			     __func__, can_skb_prv(skb)->ifindex,
			     skcb->addr.sa);
		return;
	}

	/* TODO: abort RTS when a similar
	 * TP is pending in the other direction
	 */
	session = j1939_session_get_by_skb(priv, j1939_sessionq(priv, extd),
					   skb, false);
	if (session && !j1939_tp_im_transmitter(skb)) {
		/* RTS on pending connection */
		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);

		if (pgn != session->skcb->addr.pgn &&
		    dat[0] != J1939_TP_CMD_BAM)
			j1939_xtp_tx_abort(skb, extd, true,
					   J1939_XTP_ABORT_BUSY, pgn);

		goto out_session_put;
	} else if (!session && j1939_tp_im_transmitter(skb)) {
		netdev_alert(priv->ndev, "%s: I should tx (%i %02x %02x)\n",
			     __func__, can_skb_prv(skb)->ifindex,
			     skcb->addr.sa, skcb->addr.da);

		return;
	}

	if (session && session->last_cmd != 0) {
		/* we received a second rts on the same connection */
		netdev_alert(priv->ndev, "%s: connection exists (%i %02x %02x)\n",
			     __func__, can_skb_prv(skb)->ifindex, skcb->addr.sa,
			     skcb->addr.da);

		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);

		goto out_session_put;
	}

	if (session) {
		/* make sure 'sa' & 'da' are correct !
		 * They may be 'not filled in yet' for sending
		 * skb's, since they did not pass the Address Claim ever.
		 */
		session->skcb->addr.sa = skcb->addr.sa;
		session->skcb->addr.da = skcb->addr.da;
	} else {
		enum j1939_xtp_abort abort = J1939_XTP_ABORT_NO_ERROR;

		if (extd) {
			len = j1939_etp_ctl_to_size(dat);
			if (len > J1939_MAX_ETP_PACKET_SIZE)
				abort = J1939_XTP_ABORT_FAULT;
			else if (j1939_tp_max_packet_size &&
				 (len > j1939_tp_max_packet_size))
				abort = J1939_XTP_ABORT_RESOURCE;
			else if (len <= J1939_MAX_TP_PACKET_SIZE)
				abort = J1939_XTP_ABORT_FAULT;
		} else {
			len = j1939_tp_ctl_to_size(dat);
			if (len > J1939_MAX_TP_PACKET_SIZE)
				abort = J1939_XTP_ABORT_FAULT;
			else if (j1939_tp_max_packet_size &&
				 (len > j1939_tp_max_packet_size))
				abort = J1939_XTP_ABORT_RESOURCE;
		}
		if (abort) {
			j1939_xtp_tx_abort(skb, extd, true, abort, pgn);
			return;
		}

		session = j1939_session_fresh_new(priv, len, skb, pgn);
		if (!session) {
			j1939_xtp_tx_abort(skb, extd, true,
					   J1939_XTP_ABORT_RESOURCE, pgn);
			return;
		}
		session->extd = extd;

		/* initialize the control buffer: plain copy */
		session->pkt.total = (len + 6) / 7;
		session->pkt.block = 0xff;
		if (!extd) {
			if (dat[3] != session->pkt.total)
				netdev_alert(priv->ndev, "%s: strange total, %u != %u\n",
					     __func__, session->pkt.total,
					     dat[3]);
			session->pkt.total = dat[3];
			session->pkt.block = min(dat[3], dat[4]);
		}

		session->pkt.done = 0;
		session->pkt.tx = 0;

		WARN_ON_ONCE(j1939_session_insert(session));
	}
	session->last_cmd = dat[0];

	j1939_tp_set_rxtimeout(session, 1250);

	if (j1939_tp_im_receiver(session->skb)) {
		if (extd || dat[0] != J1939_TP_CMD_BAM)
			j1939_tp_schedule_txtimer(session, 0);
	}

	/* as soon as it's inserted, things can go fast
	 * protect against a long delay
	 * between spin_unlock & next statement
	 * so, only release here, at the end
	 */
 out_session_put:
	j1939_session_put(session);
}

static void j1939_xtp_rx_dpo(struct j1939_priv *priv, struct sk_buff *skb,
			     bool extd)
{
	struct j1939_session *session;
	pgn_t pgn;
	const u8 *dat = skb->data;

	pgn = j1939_xtp_ctl_to_pgn(dat);
	session = j1939_session_get_by_skb(priv, j1939_sessionq(priv, extd),
					   skb, false);
	if (!session) {
		netdev_info(priv->ndev, "%s: no connection found\n", __func__);
		return;
	}

	if (session->skcb->addr.pgn != pgn) {
		netdev_info(priv->ndev, "%s: different pgn\n", __func__);
		j1939_xtp_tx_abort(skb, true, true, J1939_XTP_ABORT_BUSY, pgn);
		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);
		goto out_session_put;
	}

	/* transmitted without problems */
	session->pkt.dpo = j1939_etp_ctl_to_packet(skb->data);
	session->last_cmd = dat[0];
	j1939_tp_set_rxtimeout(session, 750);
 out_session_put:
	j1939_session_put(session);
}

static void j1939_xtp_rx_dat(struct j1939_priv *priv, struct sk_buff *skb,
			     bool extd)
{
	struct j1939_session *session;
	const u8 *dat;
	u8 *tpdat;
	int offset;
	int nbytes;
	bool final = false;
	bool do_cts_eoma = false;
	int packet;

	session = j1939_session_get_by_skb(priv, j1939_sessionq(priv, extd),
					   skb, false);
	if (!session) {
		netdev_info(priv->ndev, "%s: no connection found\n", __func__);
		return;
	}
	dat = skb->data;
	if (skb->len <= 1)
		/* makes no sense */
		goto out_session_cancel;

	j1939_session_lock(session);

	switch (session->last_cmd) {
	case 0xff:
		break;
	case J1939_ETP_CMD_DPO:
		if (extd)
			break;
	case J1939_TP_CMD_BAM:
	case J1939_TP_CMD_CTS:
		if (!extd)
			break;
	default:
		netdev_info(priv->ndev, "%s: last %02x\n", __func__,
			    session->last_cmd);
		goto out_session_unlock;
	}

	packet = (dat[0] - 1 + session->pkt.dpo);
	offset = packet * 7;
	if (packet > session->pkt.total ||
	    (session->pkt.done + 1) > session->pkt.total) {
		netdev_info(priv->ndev, "%s: should have been completed\n",
			    __func__);
		goto out_session_unlock;
	}
	nbytes = session->skb->len - offset;
	if (nbytes > 7)
		nbytes = 7;
	if (nbytes <= 0 || (nbytes + 1) > skb->len) {
		netdev_info(priv->ndev, "%s: nbytes %i, len %i\n", __func__,
			    nbytes, skb->len);
		goto out_session_unlock;
	}
	tpdat = session->skb->data;
	memcpy(&tpdat[offset], &dat[1], nbytes);
	if (packet == session->pkt.done)
		++session->pkt.done;

	if (!extd && j1939_cb_is_broadcast(session->skcb)) {
		if (session->pkt.done >= session->pkt.total)
			final = true;
	} else {
		/* never final, an EOMA must follow */
		if (session->pkt.done >= session->pkt.last)
			do_cts_eoma = true;
	}
	j1939_session_unlock(session);

	if (final) {
		j1939_session_completed(session);
	} else if (do_cts_eoma) {
		j1939_tp_set_rxtimeout(session, 1250);
		if (j1939_tp_im_receiver(session->skb))
			j1939_tp_schedule_txtimer(session, 0);
	} else {
		j1939_tp_set_rxtimeout(session, 250);
	}
	session->last_cmd = 0xff;
	j1939_session_put(session);

	return;

 out_session_unlock:
	/* unlock session (spinlock) before trying to send */
	j1939_session_unlock(session);
 out_session_cancel:
	j1939_session_timers_cancel(session);
	j1939_session_cancel(session, J1939_XTP_ABORT_FAULT);
	j1939_session_put(session);
}

static inline int j1939_tp_tx_initial(struct j1939_session *session)
{
	int ret;

	ret = j1939_tp_txnext(session);
	/* set nonblocking for further packets */
	session->skcb->msg_flags |= MSG_DONTWAIT;

	return ret;
}

/* j1939 main intf */
int j1939_tp_send(struct j1939_priv *priv, struct sk_buff *skb)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	struct j1939_session *session;
	int ret;

	if (skcb->addr.pgn == J1939_TP_PGN_DAT ||
	    skcb->addr.pgn == J1939_TP_PGN_CTL ||
	    skcb->addr.pgn == J1939_ETP_PGN_DAT ||
	    skcb->addr.pgn == J1939_ETP_PGN_CTL)
		/* avoid conflict */
		return -EDOM;
	if (skb->len > J1939_MAX_ETP_PACKET_SIZE ||
	    (j1939_tp_max_packet_size && skb->len > j1939_tp_max_packet_size))
		return -EMSGSIZE;
	if (skb->len > J1939_MAX_TP_PACKET_SIZE && j1939_cb_is_broadcast(skcb))
		return -EDESTADDRREQ;

	/* fill in addresses from names */
	ret = j1939_ac_fixup(priv, skb);
	if (unlikely(ret))
		return ret;

	/* fix dst_flags, it may be used there soon */
	if (j1939_address_is_unicast(skcb->addr.da) &&
	    priv->ents[skcb->addr.da].nusers)
		skcb->dst_flags |= J1939_ECU_LOCAL;

	/* src is always local, I'm sending ... */
	skcb->src_flags |= J1939_ECU_LOCAL;

	/* prepare new session */
	session = j1939_session_new(priv, skb);
	if (!session)
		return -ENOMEM;

	if (skb->len > J1939_MAX_TP_PACKET_SIZE)
		session->extd = J1939_EXTENDED;
	else
		session->extd = J1939_REGULAR;
	session->transmission = true;
	session->pkt.total = (skb->len + 6) / 7;
	session->pkt.block = session->extd ? 255 :
		min(j1939_tp_block ?: 255, session->pkt.total);

	if (j1939_cb_is_broadcast(session->skcb))
		/* set the end-packet for broadcast */
		session->pkt.last = session->pkt.total;

	/* insert into queue, but avoid collision with pending session */
	if (session->skcb->msg_flags & MSG_DONTWAIT)
		ret = j1939_session_insert(session);
	else
		ret = wait_event_interruptible(priv->tp_wait,
					       j1939_session_insert(session) == 0);
	if (ret < 0)
		goto failed;

	ret = j1939_tp_tx_initial(session);
	if (ret)
		goto failed;

	/* transmission started */
	j1939_session_put(session);
	return 0;

 failed:
	j1939_session_timers_cancel(session);
	j1939_session_cancel(session, J1939_XTP_ABORT_NO_ERROR);
	j1939_session_put(session);
	return ret;
}

int j1939_tp_recv(struct j1939_priv *priv, struct sk_buff *skb)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	const u8 *dat;

	if (!j1939_tp_im_involved_anydir(skb))
		return 0;

	switch (skcb->addr.pgn) {
	case J1939_ETP_PGN_DAT:
		j1939_xtp_rx_dat(priv, skb, J1939_EXTENDED);
		break;
	case J1939_ETP_PGN_CTL:
		if (skb->len < 8) {
			j1939_xtp_rx_bad_message(priv, skb, J1939_EXTENDED);
			break;
		}
		dat = skb->data;
		switch (*dat) {
		case J1939_ETP_CMD_RTS:
			j1939_xtp_rx_rts(priv, skb, J1939_EXTENDED);
			break;
		case J1939_ETP_CMD_CTS:
			j1939_xtp_rx_cts(priv, skb, J1939_EXTENDED);
			break;
		case J1939_ETP_CMD_DPO:
			j1939_xtp_rx_dpo(priv, skb, J1939_EXTENDED);
			break;
		case J1939_ETP_CMD_EOMA:
			j1939_xtp_rx_eoma(priv, skb, J1939_EXTENDED);
			break;
		case J1939_ETP_CMD_ABORT:
			j1939_xtp_rx_abort(priv, skb, J1939_EXTENDED);
			break;
		default:
			j1939_xtp_rx_bad_message(priv, skb, J1939_EXTENDED);
			break;
		}
		break;
	case J1939_TP_PGN_DAT:
		j1939_xtp_rx_dat(priv, skb, J1939_REGULAR);
		break;
	case J1939_TP_PGN_CTL:
		if (skb->len < 8) {
			j1939_xtp_rx_bad_message(priv, skb, J1939_REGULAR);
			break;
		}
		dat = skb->data;
		switch (*dat) {
		case J1939_TP_CMD_BAM:
		case J1939_TP_CMD_RTS:
			j1939_xtp_rx_rts(priv, skb, J1939_REGULAR);
			break;
		case J1939_TP_CMD_CTS:
			j1939_xtp_rx_cts(priv, skb, J1939_REGULAR);
			break;
		case J1939_TP_CMD_EOMA:
			j1939_xtp_rx_eoma(priv, skb, J1939_REGULAR);
			break;
		case J1939_TP_CMD_ABORT:
			j1939_xtp_rx_abort(priv, skb, J1939_REGULAR);
			break;
		default:
			j1939_xtp_rx_bad_message(priv, skb, J1939_REGULAR);
			break;
		}
		break;
	default:
		return 0; /* no problem */
	}
	return 1; /* "I processed the message" */
}

int j1939_tp_rmdev_notifier(struct j1939_priv *priv)
{
	struct j1939_session *session, *saved;
	struct net_device *ndev = priv->ndev;

	j1939_session_list_lock(priv);
	list_for_each_entry_safe(session, saved,
				 &priv->tp_sessionq, list) {
		if (can_skb_prv(session->skb)->ifindex != ndev->ifindex)
			continue;
		j1939_session_timers_cancel(session);
	}
	list_for_each_entry_safe(session, saved,
				 &priv->tp_extsessionq, list) {
		if (can_skb_prv(session->skb)->ifindex != ndev->ifindex)
			continue;
		j1939_session_timers_cancel(session);
	}
	j1939_session_list_unlock(priv);
	return NOTIFY_DONE;
}

void j1939_tp_init(struct j1939_priv *priv)
{
	spin_lock_init(&priv->tp_session_list_lock);
	INIT_LIST_HEAD(&priv->tp_sessionq);
	INIT_LIST_HEAD(&priv->tp_extsessionq);
	init_waitqueue_head(&priv->tp_wait);
}
