#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/gpio_const.h>
#include <linux/of.h>
#include <linux/of_irq.h>

/* ioctl command, use to open or close device */
#define AW8155_OPEN                     _IOC(_IOC_NONE, 0x07, 0x03, 0)
#define AW8155_CLOSE                    _IOC(_IOC_NONE, 0x07, 0x04, 0)
/* awinic aw8155 pa control pin               */
#define AW8155_CONTROL_PIN              GPIO21
/* log tag of aw8155 pa                       */
#define AW8155_LOG_TAG                  "aw8155"
/* enable debug information                   */
#define AW8155_DEBUG
#ifdef AW8155_DEBUG
#define AW8155_LOGD(format, args...)    do                              \
                {                                                       \
                    printk(KERN_DEBUG AW8155_LOG_TAG format, ##args);   \
                }while(0)
#define AW8155_LOGE(format, args...)    do                              \
                {                                                       \
                    printk(KERN_ERR AW8155_LOG_TAG format, ##args);     \
                }while(0)
#else
#define AW8155_LOGD(format, args...)    do{}while(0)
#define AW8155_LOGE(format, args...)    do{}while(0)
#endif

static dev_t aw8155_devno;
static struct cdev aw8155_cdev;
static struct class *aw8155_class = NULL;
static struct device *aw8155_device = NULL;
struct pinctrl *aw8155_pinctrl;
struct pinctrl_state *pa_ctrl_pin;

/* xCLASS : D/AB class PA
 * number : PA magnification
 * xDxW or xW : Speaker Power, such as 0.65W = 0D65W
 * RNS : RF TDD Noise Suppression
 * MUTE : Mute Technology                               */
typedef enum {
    AW8155_MODE_MIX = 0,
    AW8155_MODE_DCLASS_8_0D65W_RNS = 1,
    AW8155_MODE_DCLASS_12_0D85W_RNS,
    AW8155_MODE_DCLASS_12_0D85W_RNS_MUTE,
    AW8155_MODE_ABCLASS_12_NULL_RNS,
    AW8155_MODE_CLOSE,
    AW8155_MODE_MAX,
}aw8155_mode;

static int aw8155_io_config(struct platform_device *dev)
{
    int err = 0;
    struct pinctrl_state *pins_default;
    AW8155_LOGD("+%s\n", __func__);

    /* get pin config from device tree */
    aw8155_pinctrl = devm_pinctrl_get(&dev->dev);
    if (IS_ERR(aw8155_pinctrl)) {
        AW8155_LOGE("devm_pinctrl_get error\n");
        return -1;
    }
    pins_default = pinctrl_lookup_state(aw8155_pinctrl, "default");
    if (IS_ERR(pins_default)) {
        AW8155_LOGE("pinctrl_lookup_state default error\n");
    }
    pa_ctrl_pin = pinctrl_lookup_state(aw8155_pinctrl, "en_pin");
    if (IS_ERR(pa_ctrl_pin)) {
        AW8155_LOGE("pinctrl_lookup_state en_pin error\n");
        return -1;
    } else {
        pinctrl_select_state(aw8155_pinctrl, pa_ctrl_pin);
    }
    /* init the aw8155 enable pin */
    err = gpio_request(AW8155_CONTROL_PIN, "aw8155_en");
    if (err) {
        AW8155_LOGE("+%s request gpio error !\n", __func__);
        return err;
    }
    gpio_direction_output(AW8155_CONTROL_PIN, 0);

    return 0;
}

static int aw8155_set_mode(aw8155_mode mode)
{
    int i;
    
    AW8155_LOGD("+%s\n", __func__);
    
    if ((mode >= AW8155_MODE_MAX)
        || (mode <= AW8155_MODE_MIX)) {
        AW8155_LOGE("mode argument is error !\n");
        return -1;
    }

    if (mode == AW8155_MODE_CLOSE) {
        gpio_set_value(AW8155_CONTROL_PIN, 0);
        msleep(1);                      // sleep 1ms, make sure the aw8155 is closed
    } else {
        msleep(1);                      // 1ms use to close the mode before set it
        for (i = 0; i < mode; ++i)
        {
            ndelay(2000);               // 2us
            gpio_set_value(AW8155_CONTROL_PIN, 0);
            ndelay(2000);               // 2us
            gpio_set_value(AW8155_CONTROL_PIN, 1);
        }
        msleep(45);                     // 42ms to 50ms, make sure the mode is built
    }

    return 0;
}

static int aw8155_ops_open(struct inode *inode, struct file *file)
{
    AW8155_LOGD("+%s\n", __func__);
    return 0;
}

static int aw8155_ops_release(struct inode *inode, struct file *file)
{
   AW8155_LOGD("+%s\n", __func__);
    return 0;
}

static long aw8155_ops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    AW8155_LOGD("+%s, cmd = %d, arg = %ld\n", __func__, cmd, arg);

    switch (cmd)
    {
    case AW8155_OPEN:
        AW8155_LOGD("+%s, open\n", __func__);
        ret = aw8155_set_mode((aw8155_mode)arg);
        break;
    case AW8155_CLOSE:
        AW8155_LOGD("+%s, close\n", __func__);
        ret = aw8155_set_mode((aw8155_mode)arg);
        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}

static struct file_operations aw8155_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = aw8155_ops_ioctl,
    .open           = aw8155_ops_open,
    .release        = aw8155_ops_release,
};

struct of_device_id aw8155_of_match[] = {
    { .compatible = "mediatek,aw8155-pa", },
    {},
};

static int aw8155_drv_probe(struct platform_device *dev)
{
    int ret = 0, err = 0;

    AW8155_LOGD("+%s\n", __func__);

    if (aw8155_io_config(dev) != 0) {
        AW8155_LOGE("pa control pin init fail\n");
        return -1;
    }

    ret = alloc_chrdev_region(&aw8155_devno, 0, 1, "aw8155");
    if (ret) {
        AW8155_LOGE("alloc_chrdev_region fail: %d\n", ret);
        goto aw8155_probe_error;
    } else {
        AW8155_LOGD("major: %d, minor: %d\n", MAJOR(aw8155_devno), MINOR(aw8155_devno));
    }
    cdev_init(&aw8155_cdev, &aw8155_fops);
    aw8155_cdev.owner = THIS_MODULE;
    err = cdev_add(&aw8155_cdev, aw8155_devno, 1);
    if (err) {
        AW8155_LOGE("cdev_add fail: %d\n", err);
        goto aw8155_probe_error;
    }

    aw8155_class = class_create(THIS_MODULE, "aw8155");
    if (IS_ERR(aw8155_class)) {
        AW8155_LOGE("Unable to create class\n");
        goto aw8155_probe_error;
    }

    aw8155_device = device_create(aw8155_class, NULL, aw8155_devno, NULL, "aw8155");
    if(aw8155_device == NULL) {
        AW8155_LOGE("device_create fail\n");
        goto aw8155_probe_error;
    }

    return 0;

aw8155_probe_error:
    if (err == 0)
        cdev_del(&aw8155_cdev);
    if (ret == 0)
        unregister_chrdev_region(aw8155_devno, 1);

    return -1;
}

static int aw8155_drv_remove(struct platform_device *dev)
{
    AW8155_LOGD("+%s\n", __func__);
    cdev_del(&aw8155_cdev);
    unregister_chrdev_region(aw8155_devno, 1);
    device_destroy(aw8155_class, aw8155_devno);
    class_destroy(aw8155_class);

    return 0;
}

static struct platform_driver aw8155_platform_driver =
{
    .probe      = aw8155_drv_probe,
    .remove     = aw8155_drv_remove,
    .driver     = {
        .name   = "aw8155",
        .of_match_table = aw8155_of_match,
    },
};

static int __init aw8155_init(void)
{
    int ret = 0;
    AW8155_LOGD("+%s\n", __func__);

    ret = platform_driver_register(&aw8155_platform_driver);
    if(ret) {
        AW8155_LOGE("platform_driver_register fail\n");
        return ret;
    }
    return ret;
}

static void __exit aw8155_exit(void)
{
    AW8155_LOGD("+%s\n", __func__);
    platform_driver_unregister(&aw8155_platform_driver);
}

module_init(aw8155_init);
module_exit(aw8155_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wingtech Multimedia Group");
MODULE_DESCRIPTION("aw8155 control Driver");
