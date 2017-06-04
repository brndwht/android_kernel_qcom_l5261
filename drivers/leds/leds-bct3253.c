
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <asm/errno.h> 
#include <linux/cdev.h>
#include <linux/leds.h>

#define BCT_I2C_NAME			"bct3253"

static DEFINE_MUTEX(leds_mutex);

#define  BCT_DEBUG
struct i2c_client *bct3253_client;
enum cust_led_id
{
	CUST_LED_RED,
	CUST_LED_GREEN,
	CUST_LED_BLUE,	
	CUST_LED_TOTAL
};

struct cust_led_data{
	char                 *name;
	enum cust_led_id  led_id;
};

struct BCT_led_data {
	struct led_classdev     cdev;
	struct cust_led_data  cust_data;
	struct work_struct	 work;
	int level;
	int delay_on;
	int delay_off;
};

#define  LED_COUNT   2
struct cust_led_data cust_led_list[LED_COUNT] =
{
	{"red", CUST_LED_RED},
	{"green", CUST_LED_GREEN},
};

struct BCT_led_data *g_BCT_data[LED_COUNT];

int  const  leds_count = sizeof(cust_led_list);

int led_status = 0;
int g_led_suspend = 0;


#define TIME_STEP        500  //500ms
#define  LED_CURRENT  0x32		//defaut  02h  IMAX[1:0] = 01   current  0x32 x 0.1 = 5mA

int  bct3253_translate_timer(int reg ,unsigned long delay_on, unsigned long delay_off)
{
	int bright = 0;
	int dark = 0;	
	if(delay_on==0 && delay_off ==0)
		return -1;

	if(delay_on < TIME_STEP)
		delay_on = TIME_STEP;

	if(delay_off < TIME_STEP)
		delay_off = TIME_STEP;
	
       bright =  delay_on/TIME_STEP;
       dark   =  (delay_off/ TIME_STEP)<<4;
	i2c_smbus_write_byte_data(bct3253_client, reg, dark|bright);
	
	#ifdef BCT_DEBUG
		pr_info("wangyanhui   bct3253_translate_timer    dark|bright = 0x%x \n",  dark|bright);	
	#endif
	
	return 0;
}

void bct3253_led_sleep_mode(void){
}
void bct3253_led_enable_mode(void){
}

void bct3253_led_off(void){
	i2c_smbus_write_byte_data(bct3253_client, 0x00, 0x01);
}

void bct3253_red_led_blink(unsigned long delay_on, unsigned long delay_off);
void bct3253_red_led_on(void)
{
	i2c_smbus_write_byte_data(bct3253_client, 0x00, 0x01);	//reset chip
	i2c_smbus_write_byte_data(bct3253_client, 0x04, LED_CURRENT);
	i2c_smbus_write_byte_data(bct3253_client, 0x01, 0x02);	

	#ifdef BCT_DEBUG
		pr_info("wangyanhui  bct3253_red_led_on      led_status = %x  \n",  led_status);
	#endif
}
void bct3253_red_led_off(void)
{
	i2c_smbus_write_byte_data(bct3253_client, 0x01, 0x00);
	mdelay(1);
	#ifdef BCT_DEBUG
		pr_info("wangyanhui  bct3253_red_led_off     led_status = %x  \n",  led_status);
	#endif
}
void bct3253_green_led_blink(unsigned long delay_on, unsigned long delay_off);
void bct3253_green_led_on(void)
{
	i2c_smbus_write_byte_data(bct3253_client, 0x00, 0x01);
	i2c_smbus_write_byte_data(bct3253_client, 0x03, LED_CURRENT);
	i2c_smbus_write_byte_data(bct3253_client, 0x01, 0x01);	

	#ifdef BCT_DEBUG
		pr_info("wangyanhui  bct3253_green_led_on     led_status  = %x  \n",  led_status);
	#endif
}

void bct3253_green_led_off(void)
{
	i2c_smbus_write_byte_data(bct3253_client, 0x01, 0x00);	
	mdelay(1);

	#ifdef BCT_DEBUG
		pr_info("wangyanhui  bct3253_green_led_off      led_status = %x  \n",  led_status);
	#endif
}

void bct3253_red_led_blink(unsigned long delay_on, unsigned long delay_off)
{
	int ret=0;
	i2c_smbus_write_byte_data(bct3253_client, 0x00, 0x01);
	ret = bct3253_translate_timer(0x07, delay_on, delay_off);
	if(ret < 0)
	{
		bct3253_red_led_off();
		return;
	}
	i2c_smbus_write_byte_data(bct3253_client, 0x0f, 0x00);
	i2c_smbus_write_byte_data(bct3253_client, 0x10, 0x00);	
	i2c_smbus_write_byte_data(bct3253_client, 0x04, LED_CURRENT);
	i2c_smbus_write_byte_data(bct3253_client, 0x01, 0x22);

}

void bct3253_green_led_blink(unsigned long delay_on, unsigned long delay_off)
{
	int ret=0;
	i2c_smbus_write_byte_data(bct3253_client, 0x00, 0x01);
	ret = bct3253_translate_timer(0x06 ,delay_on, delay_off);
	if(ret < 0)
	{
		bct3253_green_led_off();
		return;
	}
	i2c_smbus_write_byte_data(bct3253_client, 0x0b, 0x00);
	i2c_smbus_write_byte_data(bct3253_client, 0x0c, 0x00);	
	i2c_smbus_write_byte_data(bct3253_client, 0x03, LED_CURRENT);
	i2c_smbus_write_byte_data(bct3253_client, 0x01, 0x11);
}

void BCT_cust_led_set_cust(struct cust_led_data *cust, int level)
{
#ifdef BCT_DEBUG
	pr_info("wangyanhui  BCT_cust_led_set_cust    level = %d  cust->led_id = %d  \n", level , cust->led_id);
#endif
	if(level > 0)
	{
		switch(cust->led_id)
		{
			case  CUST_LED_RED:
				bct3253_red_led_on();
				break;
			case  CUST_LED_GREEN:
				bct3253_green_led_on();
				break;

			default:
				break;
		}
	}
	else
	{
		switch(cust->led_id)
		{
			case  CUST_LED_RED:
				bct3253_red_led_off();
				break;
			case  CUST_LED_GREEN:
				bct3253_green_led_off();
				break;

			default:
				break;
		}
	}
}
static void BCT_led_set(struct led_classdev *led_cdev, enum led_brightness level)
{
	struct BCT_led_data *led_data =
		container_of(led_cdev, struct BCT_led_data, cdev);

	if (level < LED_OFF) {
		return;
	}

	if (level > led_data->cdev.max_brightness)
		level = led_data->cdev.max_brightness;
	
	led_data->cdev.brightness = level;
	schedule_work(&led_data->work);
}

void BCT_led_blink_set(struct BCT_led_data *led_data)
{

#ifdef BCT_DEBUG
	pr_info("wangyanhui  BCT_blink_set  led_data->cust_data.led_id = %d \n" ,led_data->cust_data.led_id);
	pr_info("wangyanhui  BCT_blink_set  led_data->delay_on = %d  led_data->delay_off = %d\n" ,led_data->delay_on ,led_data->delay_off);
#endif
	switch(led_data->cust_data.led_id)
	{
		case CUST_LED_RED:
			bct3253_red_led_blink(led_data->delay_on, led_data->delay_off);
			break;
		case CUST_LED_GREEN:
			bct3253_green_led_blink(led_data->delay_on, led_data->delay_off);
			break;
		default:
			break;
	}
}

static int  BCT_blink_set(struct led_classdev *led_cdev,
			     unsigned long *delay_on,
			     unsigned long *delay_off)
{
	struct BCT_led_data *led_data =
		container_of(led_cdev, struct BCT_led_data, cdev);

	led_data->delay_on = *delay_on;
	led_data->delay_off = *delay_off;
	//pr_info("wangyanhui  BCT_blink_set  delay_on = %x  delay_off = %x  ",led_data->delay_on ,led_data->delay_off);	
	BCT_led_blink_set(led_data);
	
	return 0;
}


void BCT_led_work(struct work_struct *work)
{
	struct BCT_led_data *led_data =
		container_of(work, struct BCT_led_data, work);

	mutex_lock(&leds_mutex);
	BCT_cust_led_set_cust(&led_data->cust_data, led_data->cdev.brightness);
	mutex_unlock(&leds_mutex);
}

static int bct3253_probe(struct i2c_client *client, const struct i2c_device_id *id){

	int ret;
	int err = 0;
	int i = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}
	
	bct3253_client = client;
	
	i2c_smbus_write_byte_data(bct3253_client, 0x00, 0x01); //reset chip
	mdelay(1);
	ret = i2c_smbus_write_byte_data(bct3253_client, 0x01, 0x00);//turn off leds	

	if(ret < 0){
		printk("can't find bct3253 led control ic!");
		err = ret;
		goto exit0;
	}

	for(i = 0; i < LED_COUNT; i++)
	{
		g_BCT_data[i] = kzalloc(sizeof(struct BCT_led_data), GFP_KERNEL);
		if (!g_BCT_data[i]) {
			dev_err(&client->dev,
					"%s: memory allocation failed.", __func__);
			err = -ENOMEM;
			goto exit0;
		}

		g_BCT_data[i]->cust_data.name = cust_led_list[i].name;
		g_BCT_data[i]->cust_data.led_id = cust_led_list[i].led_id;
		g_BCT_data[i]->cdev.name = cust_led_list[i].name;
		g_BCT_data[i]->cdev.brightness_set = BCT_led_set;
		g_BCT_data[i]->cdev.blink_set =BCT_blink_set;

		INIT_WORK(&g_BCT_data[i]->work, BCT_led_work);

		ret = led_classdev_register((struct device *)&client->dev , &g_BCT_data[i]->cdev);
	}

exit0:
	return err;
}

static int bct3253_remove(struct i2c_client *client){

	int i;
	for (i = 0; i < LED_COUNT; i++) {
		led_classdev_unregister(&g_BCT_data[i]->cdev);
		cancel_work_sync(&g_BCT_data[i]->work);
		kfree(g_BCT_data[i]);
		g_BCT_data[i] = NULL;
	}
		
	return 0;
}
	

void bct3253_shutdown(struct i2c_client *client)
{
	mutex_lock(&leds_mutex);
	bct3253_led_off();
	mutex_unlock(&leds_mutex);
}


static int bct3253_suspend(struct i2c_client *client, pm_message_t mesg)
{
#ifdef BCT_DEBUG
	pr_info("wangyanhui  bct3253_suspend    led_status = %x  \n", led_status);
#endif
	//if(led_status == 0)
	//{
		//bct3253_led_sleep_mode();
		//g_led_suspend = 1;
	//}
	return 0;
}

static int bct3253_resume(struct i2c_client *client)
{
	//if(g_led_suspend)
	//{
		//bct3253_led_enable_mode();
		//g_led_suspend = 0;
	//}
	return 0;
}



static const struct i2c_device_id bct3253_id[] = {
	{BCT_I2C_NAME, 0},
	{ }
};


static struct of_device_id bct3253_match_table[] = {
        { .compatible = "bct,bct3253",},
        { },
};

/* 1. 分配一个i2c_driver结构体 */
/* 2. 设置i2c_driver结构体 */
static struct i2c_driver bct3253_driver = {
	.driver = {
		.name	= BCT_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bct3253_match_table,
	},
	.probe = bct3253_probe,
	.remove = bct3253_remove,
	.suspend    = bct3253_suspend,
	.resume     = bct3253_resume,	
	.shutdown = bct3253_shutdown,
	.id_table = bct3253_id,
};

static int __init bct3253_init(void)
{
	int err;
	printk("%s\n",__func__);
	err = i2c_add_driver(&bct3253_driver);
	if (err) {
		printk(KERN_WARNING "bct3253 driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          bct3253_driver.driver.name);
	}
	return err;
}

static void __exit bct3253_exit(void)
{
	printk("%s\n",__func__);
	i2c_del_driver(&bct3253_driver);
}

module_init(bct3253_init);
module_exit(bct3253_exit);

MODULE_AUTHOR("tinno.com");
MODULE_LICENSE("GPL");

