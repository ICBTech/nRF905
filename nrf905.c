#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>


static struct class *nrf905_class = NULL;
static struct device *nrf905_device = NULL;
static int nrf905_major;

struct nrf905_drvdata {
    dev_t devt;
    spinlock_t spi_lock;
    struct spi_device *spi;
    struct list_head device_entry;
    unsigned users;
    int gpio_pwr_up;
    int gpio_trx_ce;
    int gpio_tx_en;
    int gpio_dr;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DEFINE_MUTEX(nrf905_cdev_lock);
static DECLARE_WAIT_QUEUE_HEAD(nrf905_cdev_wq);


/* ************************************************************************** *
 * Low-level communication with the chip                                      *
 * ************************************************************************** */


static int nrf905_spi_sync(struct spi_device *spi, uint8_t *txbytes, uint8_t *rxbytes, uint8_t n) {
    struct spi_message msg;
    struct spi_transfer xfer;

    spi_message_init(&msg);
    memset(&xfer, 0, sizeof(xfer));
    xfer.tx_buf = txbytes;
    xfer.rx_buf = rxbytes;
    xfer.len = n;
    xfer.bits_per_word = 0;
    spi_message_add_tail(&xfer, &msg);
    return spi_sync(spi, &msg);
}


static void nrf905_spi_w_config(struct spi_device *spi, uint8_t address, uint8_t byte) {
    uint8_t txbytes[2];
    uint8_t rxbytes[sizeof(txbytes)];

    txbytes[0] = address & 0x0F;
    txbytes[1] = byte;

    nrf905_spi_sync(spi, txbytes, rxbytes, sizeof(txbytes));
}


static uint8_t nrf905_spi_r_config(struct spi_device *spi, uint8_t address) {
    uint8_t txbytes[2];
    uint8_t rxbytes[sizeof(txbytes)];

    txbytes[0] = (address & 0x0F) | 0x10;
    txbytes[1] = 0xFF; // dummy byte for reading the response

    nrf905_spi_sync(spi, txbytes, rxbytes, sizeof(txbytes));

    return rxbytes[1];
}


static void nrf905_spi_w_tx_payload(struct spi_device *spi, const uint8_t *payload, uint8_t count) {
    uint8_t txbytes[count + 1];
    uint8_t rxbytes[sizeof(txbytes)];

    txbytes[0] = 0x20;
    memcpy(&txbytes[1], payload, count);

    nrf905_spi_sync(spi, txbytes, rxbytes, sizeof(txbytes));
}


static void nrf905_spi_w_tx_address(struct spi_device *spi, const uint8_t *address, size_t count) {
    uint8_t txbytes[5];
    uint8_t rxbytes[sizeof(txbytes)];

    txbytes[0] = 0x22;
    memcpy(&txbytes[1], address, count);

    nrf905_spi_sync(spi, txbytes, rxbytes, sizeof(txbytes));
}


static void nrf905_spi_r_tx_address(struct spi_device *spi, uint8_t *address, size_t count) {
    uint8_t txbytes[5];
    uint8_t rxbytes[sizeof(txbytes)];

    txbytes[0] = 0x23;

    nrf905_spi_sync(spi, txbytes, rxbytes, sizeof(txbytes));

    memcpy(address, &rxbytes[1], count);
}


static void nrf905_spi_r_rx_payload(struct spi_device *spi, uint8_t *payload, uint8_t count) {
    uint8_t txbytes[33];
    uint8_t rxbytes[sizeof(txbytes)];

    txbytes[0] = 0x24;

    nrf905_spi_sync(spi, txbytes, rxbytes, sizeof(txbytes));

    memcpy(payload, &rxbytes[1], count);
}


static void nrf905_spi_w_rx_address(struct spi_device *spi, const uint8_t *address, size_t count) {
    int i;
    for (i = 0; i < count; i++) {
        nrf905_spi_w_config(spi, 5 + i, address[i]);
    }
}


static void nrf905_spi_r_rx_address(struct spi_device *spi, uint8_t *address, size_t count) {
    int i;
    for (i = 0; i < count; i++) {
        address[i] = nrf905_spi_r_config(spi, 5 + i);
    }
}


/* ************************************************************************** *
 * Sysfs attributes                                                           *
 * ************************************************************************** */


/** @brief Write handler for the tx_address sysfs attribute */
static ssize_t nrf905_attr_tx_address_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spi_device *spi = to_spi_device(dev);

    if (count != 4) {
        return -EINVAL;
    }

    nrf905_spi_w_tx_address(spi, buf, count);

    return count;
}

/** @brief Read handler for the tx_address sysfs attribute */
static ssize_t nrf905_attr_tx_address_read(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spi_device *spi = to_spi_device(dev);

    nrf905_spi_r_tx_address(spi, buf, 4);

    return 4;
}


/** @brief Device attribute for configuring the tx address */
static DEVICE_ATTR(tx_address, 0664, nrf905_attr_tx_address_read, nrf905_attr_tx_address_write);




/** @brief Write handler for the rx_address sysfs attribute */
static ssize_t nrf905_attr_rx_address_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spi_device *spi = to_spi_device(dev);

    if ((count < 1) || (count > 4)) {
        return -EINVAL;
    }

    nrf905_spi_w_rx_address(spi, buf, count);

    return count;
}

/** @brief Read handler for the rx_address sysfs attribute */
static ssize_t nrf905_attr_rx_address_read(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spi_device *spi = to_spi_device(dev);

    nrf905_spi_r_rx_address(spi, buf, 4);

    return 4;
}


/** @brief Device attribute for configuring the rx address */
static DEVICE_ATTR(rx_address, 0664, nrf905_attr_rx_address_read, nrf905_attr_rx_address_write);


/** @brief Write handler for the frequency sysfs attribute */
static ssize_t nrf905_attr_frequency_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spi_device *spi = to_spi_device(dev);
    uint32_t frequency;
    uint8_t hfreq_pll;
    uint16_t channel;
    uint8_t config[2];

    if (sscanf(buf, "%u", &frequency) != 1) {
        return -EINVAL;
    }

    if ((frequency >= 422400) && (frequency < 473600)) {
        hfreq_pll = 0;
        // round to 0.1MHz
        frequency = (frequency / 100) * 100;
        channel = (frequency - 422400) / 100;
    } else if ((frequency >= 844800) && (frequency < 947200)) {
        hfreq_pll = 1;
        // round to 0.2MHz
        frequency = (frequency / 200) * 200;
        channel = (frequency - 844800) / 200;
    } else {
        return -EINVAL;
    }

    dev_info(dev, "frequency: %u\n", frequency);
    dev_info(dev, "hfreq_pll: %hhu\n", hfreq_pll);
    dev_info(dev, "channel: %hu\n", channel);

    if (channel > 0x1FF) {
        dev_warn(dev, "channel %hu out of range!\n", channel);
        return -EINVAL;
    }

    config[0] = nrf905_spi_r_config(spi, 0);
    config[1] = nrf905_spi_r_config(spi, 1);

    config[0] = (config[0] & 0x00) | (channel & 0x00FF);
    config[1] = (config[1] & 0xFC) | ((channel & 0xFF00) >> 8) | (hfreq_pll << 1);

    nrf905_spi_w_config(spi, 0, config[0]);
    nrf905_spi_w_config(spi, 1, config[1]);

    return count;
}


/** @brief Read handler for the frequency sysfs attribute */
static ssize_t nrf905_attr_frequency_read(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spi_device *spi = to_spi_device(dev);
    uint32_t frequency;
    uint16_t channel;
    uint8_t hfreq_pll;
    uint8_t config[2];

    config[0] = nrf905_spi_r_config(spi, 0);
    config[1] = nrf905_spi_r_config(spi, 1);

    channel = config[0] + ((config[1] & 0x01) << 8);
    hfreq_pll = (config[1] & 0x02) >> 1;

    frequency = (422400 + channel * 100) * (1 + hfreq_pll);

    return sprintf(buf, "%u", frequency);
}


/** @brief Device attribute for configuring the frequency (ch_no + hfreq_pll together) */
static DEVICE_ATTR(frequency, 0664, nrf905_attr_frequency_read, nrf905_attr_frequency_write);


/** @brief Write handler for the pa_pwr sysfs attribute */
static ssize_t nrf905_attr_pa_pwr_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    struct spi_device *spi = to_spi_device(dev);
    uint8_t config;
    uint8_t pa_pwr;

    if (sscanf(buf, "%hhu", &pa_pwr) != 1) {
        return -EINVAL;
    }

    dev_info(dev, "pa_pwr: %hhu\n", pa_pwr);

    if (pa_pwr > 3) {
        dev_warn(dev, "pa_pwr %hhu out of range!\n", pa_pwr);
        return -EINVAL;
    }

    config = nrf905_spi_r_config(spi, 1);

    config = (config & 0xF3) | (pa_pwr << 2);

    nrf905_spi_w_config(spi, 1, config);

    return count;
}


/** @brief Read handler for the pa_pwr sysfs attribute */
static ssize_t nrf905_attr_pa_pwr_read(struct device *dev, struct device_attribute *attr, char *buf) {
    struct spi_device *spi = to_spi_device(dev);
    uint8_t config;
    uint8_t pa_pwr;

    config = nrf905_spi_r_config(spi, 1);

    pa_pwr = (config & 0x0C) >> 2;

    return sprintf(buf, "%hhu", pa_pwr);
}


/** @brief Device attribute for configuring the power amplifier */
static DEVICE_ATTR(pa_pwr, 0664, nrf905_attr_pa_pwr_read, nrf905_attr_pa_pwr_write);


/** @brief An array containing all the sysfs attributes */
static struct attribute *nrf905_attributes[] = {
    &dev_attr_rx_address.attr,
    &dev_attr_tx_address.attr,
    &dev_attr_frequency.attr,
    &dev_attr_pa_pwr.attr,
    NULL,
};


/** @brief The attribute group for creating and destroying all the sysfs attributes at once instead of one at a time */
static struct attribute_group nrf905_attribute_group = {
    .attrs = nrf905_attributes
};


static irqreturn_t dr_irq(int irq, void *dev_id) {
    struct nrf905_drvdata *drvdata = dev_id;

    int value = gpio_get_value(drvdata->gpio_dr);

    if (value) {
        wake_up_interruptible(&nrf905_cdev_wq);
    }

    return IRQ_HANDLED;
}


static ssize_t nrf905_cdev_read(struct file *filp, char __user *buf, size_t count, loff_t *fpos) {
    struct nrf905_drvdata *drvdata = filp->private_data;
    long status;
    uint8_t rxbuf[32];

    if (count != 32) {
        return -EMSGSIZE;
    }

    mutex_lock(&nrf905_cdev_lock);

    gpio_set_value(drvdata->gpio_tx_en, 0);
    gpio_set_value(drvdata->gpio_trx_ce, 1);

    if (wait_event_interruptible(nrf905_cdev_wq, gpio_get_value(drvdata->gpio_dr))) {
        gpio_set_value(drvdata->gpio_trx_ce, 0);

        mutex_unlock(&nrf905_cdev_lock);

        return -ERESTARTSYS;
    }

    gpio_set_value(drvdata->gpio_trx_ce, 0);

    nrf905_spi_r_rx_payload(drvdata->spi, rxbuf, count);

    mutex_unlock(&nrf905_cdev_lock);

    status = copy_to_user(buf, rxbuf, count);
    if (status < 0) {
        return status;
    } else {
        return count - status;
    }
}


static ssize_t nrf905_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *fpos) {
    struct nrf905_drvdata *drvdata = filp->private_data;
    long status;
    uint8_t txbuf[32];

    if (count != 32) {
        return -EMSGSIZE;
    }

    status = copy_from_user(txbuf, buf, count);
    if (status < 0) {
        return -EINVAL;
    }

    mutex_lock(&nrf905_cdev_lock);

    nrf905_spi_w_tx_payload(drvdata->spi, txbuf, count - status);

    gpio_set_value(drvdata->gpio_tx_en, 1);
    gpio_set_value(drvdata->gpio_trx_ce, 1);

    if (wait_event_interruptible(nrf905_cdev_wq, gpio_get_value(drvdata->gpio_dr))) {
        gpio_set_value(drvdata->gpio_trx_ce, 0);

        mutex_unlock(&nrf905_cdev_lock);

        return -ERESTARTSYS;
    }

    gpio_set_value(drvdata->gpio_trx_ce, 0);

    mutex_unlock(&nrf905_cdev_lock);

    return count - status;
}


static int nrf905_cdev_open(struct inode *inode, struct file *filp) {
    struct nrf905_drvdata *drvdata;
    int err = -ENXIO;

    mutex_lock(&device_list_lock);

    list_for_each_entry(drvdata, &device_list, device_entry) {
        if (drvdata->devt == inode->i_rdev) {
            err = 0;
            break;
        }
    }

    if (err) {
        dev_warn(&drvdata->spi->dev, "nothing for minor %d\n", iminor(inode));
        goto err_find_dev;
    }

    drvdata->users++;
    filp->private_data = drvdata;
    nonseekable_open(inode, filp);

    mutex_unlock(&device_list_lock);
    return err;

err_find_dev:
    mutex_unlock(&device_list_lock);
    return err;
}


static int nrf905_cdev_release(struct inode *inode, struct file *filp) {
    struct nrf905_drvdata *drvdata;

    mutex_lock(&device_list_lock);
    drvdata = filp->private_data;
    filp->private_data = NULL;

    drvdata->users--;
    if (!drvdata->users) {
        int dofree;

        spin_lock_irq(&drvdata->spi_lock);
        dofree = (drvdata->spi == NULL);
        spin_unlock_irq(&drvdata->spi_lock);

        if (dofree) {
            kfree(drvdata);
        }
    }
    mutex_unlock(&device_list_lock);

    return 0;
}


/** @brief Character device file operations */
static struct file_operations nrf905_fops = {
    .owner = THIS_MODULE,
    .write = nrf905_cdev_write,
    .read = nrf905_cdev_read,
    .open = nrf905_cdev_open,
    .release = nrf905_cdev_release,
};


static int nrf905_probe(struct spi_device *spi) {
    struct nrf905_drvdata *drvdata;
    int err;

    dev_info(&spi->dev, "%s\n", __func__);

    drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
    if (!drvdata) {
        return -ENOMEM;
    }

    drvdata->gpio_pwr_up = of_get_named_gpio(spi->dev.of_node, "pwr_up_gpio", 0);
    drvdata->gpio_trx_ce = of_get_named_gpio(spi->dev.of_node, "trx_ce_gpio", 0);
    drvdata->gpio_tx_en = of_get_named_gpio(spi->dev.of_node, "tx_en_gpio", 0);
    drvdata->gpio_dr = of_get_named_gpio(spi->dev.of_node, "dr_gpio", 0);

    dev_info(&spi->dev, "gpio_pwr_up: %d\n", drvdata->gpio_pwr_up);
    dev_info(&spi->dev, "gpio_trx_ce: %d\n", drvdata->gpio_trx_ce);
    dev_info(&spi->dev, "gpio_tx_en: %d\n", drvdata->gpio_tx_en);
    dev_info(&spi->dev, "gpio_dr: %d\n", drvdata->gpio_dr);

    err = gpio_request_one(drvdata->gpio_pwr_up, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "pwr_up");
    if (err < 0) {
        dev_err(&spi->dev, "could not obtain pwr_up pin %d\n", drvdata->gpio_pwr_up);
        return err;
    }

    err = gpio_request_one(drvdata->gpio_trx_ce, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "trx_ce");
    if (err < 0) {
        dev_err(&spi->dev, "could not obtain trx_ce pin %d\n", drvdata->gpio_trx_ce);
        return err;
    }

    err = gpio_request_one(drvdata->gpio_tx_en, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "tx_en");
    if (err < 0) {
        dev_err(&spi->dev, "could not obtain tx_en pin %d\n", drvdata->gpio_tx_en);
        return err;
    }

    err = gpio_request_one(drvdata->gpio_dr, GPIOF_DIR_IN, "dr");
    if (err < 0) {
        dev_err(&spi->dev, "could not obtain dr pin %d\n", drvdata->gpio_dr);
        return err;
    }

    err = request_irq(gpio_to_irq(drvdata->gpio_dr), dr_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "dr_irq", drvdata);
    if (err < 0) {
        dev_err(&spi->dev, "unable to request irq for dr pin %d\n", drvdata->gpio_dr);
        return err;
    }

    gpio_set_value(drvdata->gpio_pwr_up, 0);

    // transition to PWR_DWN is not specified in the datasheet
    // 10ms should be more than enough, considering that the wakeup time is 3ms
    msleep(10);

    gpio_set_value(drvdata->gpio_pwr_up, 1);

    // transition PWR_DWN -> ST_BY mode, i.e. wake up
    msleep(3);

    spi->master->mode_bits = SPI_MODE_0;
    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;

    err = spi_setup(spi);
    if (err < 0) {
        return err;
    }

    // crystal oscillator frequency = 16MHz
    nrf905_spi_w_config(spi, 9, 0xDF);

    drvdata->spi = spi;
    spin_lock_init(&drvdata->spi_lock);

    INIT_LIST_HEAD(&drvdata->device_entry);

    mutex_lock(&device_list_lock);

    /* Create /dev and sysfs devices */
    drvdata->devt = MKDEV(nrf905_major, 0);
    nrf905_device = device_create(nrf905_class, &spi->dev, drvdata->devt, drvdata, "nrf905");
    if (IS_ERR(nrf905_device)) {
        err = PTR_ERR(nrf905_device);
        dev_err(&spi->dev, "Error %d while trying to create %s\n", err, "nrf905");
        return err;
    } else {
        list_add(&drvdata->device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);

    spi_set_drvdata(spi, drvdata);

    err = sysfs_create_group(&spi->dev.kobj, &nrf905_attribute_group);

    return 0;
}


static int nrf905_remove(struct spi_device *spi) {
    struct nrf905_drvdata *drvdata = spi_get_drvdata(spi);

    dev_info(&spi->dev, "%s\n", __func__);

    spin_lock_irq(&drvdata->spi_lock);
    drvdata->spi = NULL;
    spin_unlock_irq(&drvdata->spi_lock);

    mutex_lock(&device_list_lock);
    list_del(&drvdata->device_entry);

    sysfs_remove_group(&spi->dev.kobj, &nrf905_attribute_group);

    device_destroy(nrf905_class, nrf905_device->devt);

    mutex_unlock(&device_list_lock);

    gpio_direction_output(drvdata->gpio_dr, 0);
    free_irq(gpio_to_irq(drvdata->gpio_dr), drvdata);
    gpio_free(drvdata->gpio_dr);

    gpio_direction_output(drvdata->gpio_tx_en, 0);
    gpio_free(drvdata->gpio_tx_en);

    gpio_direction_output(drvdata->gpio_trx_ce, 0);
    gpio_free(drvdata->gpio_trx_ce);

    gpio_direction_output(drvdata->gpio_pwr_up, 0);
    gpio_free(drvdata->gpio_pwr_up);

    return 0;
}


static struct spi_driver nrf905_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "nrf905",
    },
    .probe = nrf905_probe,
    .remove = __exit_p(nrf905_remove),
};


int nrf905_init(void) {
    pr_info("%s\n", __func__);

    nrf905_class = class_create(THIS_MODULE, "nrf905");
    if (IS_ERR(nrf905_class)) {
        pr_err("failed to register device class '%s'\n", "nrf905");
        return PTR_ERR(nrf905_class);
    }

    nrf905_major = register_chrdev(0, "nrf905", &nrf905_fops);
    if (nrf905_major < 0) {
        return nrf905_major;
    }

    return spi_register_driver(&nrf905_driver);
}


void nrf905_exit(void) {
    pr_info("%s\n", __func__);

    spi_unregister_driver(&nrf905_driver);

    unregister_chrdev(nrf905_major, "nrf905");

    class_destroy(nrf905_class);
    class_unregister(nrf905_class);
}


module_init(nrf905_init);
module_exit(nrf905_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrija Prčić <andrija.prcic@icbtech.rs>");
MODULE_DESCRIPTION("Simple driver for the Nordic nRF905 chip");
