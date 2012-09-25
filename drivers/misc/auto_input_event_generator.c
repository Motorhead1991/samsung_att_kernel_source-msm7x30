/*
 * drivers/misc/auto_input_event_generator.c - auto input event generator
 *
 * Copyright (C) 2011 Samsung electronices
 * Jaecheol Kim <jc22.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/auto_input_event_generator.h>


/*
 * input event generator structures
 * ==================================================================
 * pdata : platofrm data
 *
 * key_work : key input event workqueue handler
 * touch_work : touch input event workqueue handler
 * touchkey_work : touchkey input event workqueue handler
 *
 * key_timer : key input event timer handler
 * touch_timer : touch input event timer handler
 * touchkey_timer : touchkey input event timer handler
 *
 * dev : platform device pointer
 */
struct auto_input_generator {
    struct auto_input_event_generator_platform_data *pdata;

    struct work_struct key_work;
    struct work_struct touch_work;
    struct work_struct touchkey_work;

    struct timer_list key_timer;
    struct timer_list touch_timer;
    struct timer_list touchkey_timer;

    struct platform_device *dev;
};

/*
 * global auto input generator data
 */
struct auto_input_generator *g_aig;

/*
 * input event workqueues  - key, touch, touchkey
 * =================================================================
 */
static void auto_input_generator_key_work(struct work_struct *work) {
    struct auto_input_generator *aig = container_of(work, struct auto_input_generator, key_work);
    int keyevt = DEFAULT_EVENT_COUNT_KEY;

    keyevt = aig->pdata->key_code[random32() % keyevt];

    if (aig->pdata->key_event_handler)
        aig->pdata->key_event_handler(keyevt);
    else
        printk(KERN_ERR "[%s] no key event handler\n", __func__);
};

static void auto_input_generator_touch_work(struct work_struct *work) {
    struct auto_input_generator *aig = container_of(work, struct auto_input_generator, touch_work);
    unsigned int pos_x = DEFAULT_TOUCH_X;
    unsigned int pos_y = DEFAULT_TOUCH_Y;
    unsigned int pos_str = DEFAULT_TOUCH_STR;

    if (aig->pdata->touch_matrix_x)
        pos_x = random32() % aig->pdata->touch_matrix_x;
    else
        pos_x = random32() % pos_x;

    if (aig->pdata->touch_matrix_y)
        pos_y = random32() % aig->pdata->touch_matrix_y;
    else
        pos_y = random32() % pos_y;

    if (aig->pdata->touch_matrix_str)
        pos_str = random32() % aig->pdata->touch_matrix_str;
    else
        pos_str = random32() % pos_str;

    if (aig->pdata->touch_event_handler)
        aig->pdata->touch_event_handler(pos_x, pos_y, pos_str);
    else
        printk(KERN_ERR "[%s] no touch event handler\n", __func__);

};

static void auto_input_generator_touchkey_work(struct work_struct *work) {
    struct auto_input_generator *aig = container_of(work, struct auto_input_generator, touchkey_work);
    int touchkeyevt = DEFAULT_EVENT_COUNT_TOUCHKEY;

    touchkeyevt = aig->pdata->touchkey_code[random32() % touchkeyevt];

    if (aig->pdata->touchkey_event_handler)
        aig->pdata->touchkey_event_handler(touchkeyevt);
    else
        printk(KERN_ERR "[%s] no touchkey event handler\n", __func__);
};

/*
 * timer handlers - key, touch, touchkey
 * =================================================================
 * periodically generate timer interrupt for input events
 * and not using hrtimer cause we just need ms input events.
 */
static void key_event_timer(unsigned long data)
{
    schedule_work(&g_aig->key_work);
    mod_timer(&g_aig->key_timer, (jiffies + g_aig->pdata->repeat_time_key/1000*HZ));
}

static void touch_event_timer(unsigned long data)
{
    schedule_work(&g_aig->touch_work);
    mod_timer(&g_aig->touch_timer, (jiffies + g_aig->pdata->repeat_time_touch/1000*HZ));
}

static void touchkey_event_timer(unsigned long data)
{
    schedule_work(&g_aig->touchkey_work);
    mod_timer(&g_aig->touchkey_timer, (jiffies + g_aig->pdata->repeat_time_touchkey/1000*HZ));
}

static int __devinit auto_input_generator_probe(struct platform_device *dev)
{
    struct auto_input_event_generator_platform_data *pdata = dev_get_platdata(&dev->dev);
    struct auto_input_generator *aig;

    if (!pdata) {
        dev_err(&dev->dev, "No input event generator platform data\n");
        return -EINVAL;
    }

    aig = kzalloc(sizeof(struct auto_input_generator), GFP_KERNEL);
    if (!aig) {
        dev_err(&dev->dev, "failed to allocate memory\n");
        return -ENOMEM;
    }

    aig->pdata = pdata;

    if (pdata->event & AUTO_EVENT_KEY) {
        dev_info(&dev->dev, "create key event generator \n");
        INIT_WORK(&aig->key_work, auto_input_generator_key_work);
    }
    
    if (pdata->event & AUTO_EVENT_TOUCHKEY) {
        dev_info(&dev->dev, "create touchkey event generator\n");
        INIT_WORK(&aig->touch_work, auto_input_generator_touch_work);
    }

    if (pdata->event & AUTO_EVENT_TOUCH) {
        dev_info(&dev->dev, "create touch event generator\n");
        INIT_WORK(&aig->touchkey_work, auto_input_generator_touchkey_work);
    }

    if (pdata->auto_start == 1) {
        if (pdata->event & AUTO_EVENT_KEY) {
            printk("[jjals] key event timer run\n");

            init_timer(&aig->key_timer);
            aig->key_timer.function = key_event_timer;
            aig->key_timer.expires = jiffies + (pdata->auto_start_time/1000)*HZ;
            add_timer(&aig->key_timer);
        }
        if (pdata->event & AUTO_EVENT_TOUCHKEY) {
            printk("[jjals] touchkey timer run \n");
            init_timer(&aig->touchkey_timer);
            aig->touchkey_timer.function = touchkey_event_timer;
            aig->touchkey_timer.expires = jiffies + (pdata->auto_start_time/1000)*HZ;
            add_timer(&aig->touchkey_timer);
        }
        if (pdata->event & AUTO_EVENT_TOUCH) {
            printk("[jjals] touch timer run\n");
            init_timer(&aig->touch_timer);
            aig->touch_timer.function = touch_event_timer;
            aig->touch_timer.expires = jiffies + (pdata->auto_start_time/1000)*HZ;
            add_timer(&aig->touch_timer);
        }
    }
    g_aig = aig;

    return 0;
}

static int __devexit auto_input_generator_remove(struct platform_device *dev)
{
    struct auto_input_generator *aig = platform_get_drvdata(dev);

    kfree(aig);
    platform_set_drvdata(dev, NULL);
    
    return 0;
}

#ifdef CONFIG_PM
static int auto_input_generator_suspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}

static int auto_input_generator_resume(struct platform_device *dev)
{
    return 0;
}
#else
#define auto_input_generator_suspend   NULL
#define auto_input_generator_resume    NULL
#endif

static struct platform_driver auto_input_generator_driver = {
    .probe = auto_input_generator_probe,
    .remove = __devexit_p(auto_input_generator_remove),
    .suspend = auto_input_generator_suspend,

    .resume = auto_input_generator_resume,
    .driver = {
        .name = "auto_input_event_generator",
    },
};

static int __init auto_input_generator_init(void)
{
    return platform_driver_register(&auto_input_generator_driver);
}

static void __exit auto_input_generator_exit(void)
{
    platform_driver_unregister(&auto_input_generator_driver);
}
module_init(auto_input_generator_init);
module_exit(auto_input_generator_exit);

MODULE_AUTHOR("jc22.kim@samsung.com");
MODULE_DESCRIPTION("Samsung GGSM auto input event generator");
MODULE_LICENSE("GPL");
