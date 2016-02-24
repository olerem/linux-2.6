#ifndef _ICC_CORE_H_
#define _ICC_CORE_H_

#include <linux/device.h>

#define ICC_MAX_LUNS 255

struct icc_trf;
struct icc_master;

struct icc_device {
	struct device           dev;
	struct icc_master	*iccm;
	void                    *data;
	bool                    configured;
	unsigned int		lun_number;

	int			(*rx_cb)(struct icc_device *, void *, size_t);
};

struct icc_master {
	struct device           *dev;
	struct class		*icc_class;
	int			max_data_size;

	int (*trf_alloc)(struct icc_master *, struct icc_trf *, u8, size_t);
	int (*trf_xmit)(struct icc_master *, struct icc_trf *);

	struct icc_device	lun[ICC_MAX_LUNS];
};

struct icc_driver {
	struct device_driver	driver;
	int			(*probe)(struct icc_device *);
	int			(*remove)(struct icc_device *);
	void			(*shutdown)(struct icc_device *);
};

struct icc_trf {
	void		*buf;
	void		*hdr;
	void		*data;
	size_t		size;
	size_t		data_size;
};

static inline struct icc_device *to_icc_device(struct device *dev)
{
	return dev ? container_of(dev, struct icc_device, dev) : NULL;
}

static inline struct icc_driver *to_icc_driver(struct device_driver *drv)
{
	return drv ? container_of(drv, struct icc_driver, driver) : NULL;
}

static inline void icc_set_drvdata(struct icc_device *iccd, void *data)
{
	dev_set_drvdata(&iccd->dev, data);
}

static inline void *icc_get_drvdata(struct icc_device *iccd)
{
	return dev_get_drvdata(&iccd->dev);
}

static inline void icc_set_rxcb(struct icc_device *iccd,
		int (*rx_cb)(struct icc_device *, void *, size_t))
{
	iccd->rx_cb = rx_cb;
}

static inline int icc_trf_alloc(struct icc_master *iccm,
		struct icc_trf *trf, u8 lun, size_t size)
{
	return iccm->trf_alloc(iccm, trf, lun, size);
}
static inline int icc_trf_xmit(struct icc_master *iccm,
		struct icc_trf *trf)
{
	return iccm->trf_xmit(iccm, trf);
}

int icc_register_driver(struct icc_driver *);
void icc_unregister_driver(struct icc_driver *);
void icc_add_luns(struct icc_master *iccm);

#endif /* _ICC_CORE_H_ */
