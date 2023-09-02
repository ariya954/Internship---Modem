#define RESPONSE_DELAY 1000
#define SLEEP_TIME_MS  1000
#define LED1_NODE     DT_ALIAS(led1)
#define UART_NODE     DT_ALIAS(uart2)
#define SPI_CS        DT_ALIAS(cs)
#define SPI_NODE      DT_ALIAS(spi)
#define spi_act_pin   DT_ALIAS(spiactpin)
#define UART_BUF_SIZE 1500
#define SPI_BUF_SIZE 1024
#define NUM_OF_INPUTS 8
#define NUM_OF_OUTPUTS 8

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/modem/gsm_ppp.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_conn_mgr.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <string.h>
#include "crc32.c"

LOG_MODULE_REGISTER(sample_gsm_ppp, LOG_LEVEL_DBG);

uint32_t crc; 
uint8_t rx_buf_spi[SPI_BUF_SIZE];
char rx_buf_uart[UART_BUF_SIZE];
int rx_buf_uart_pos = 0;

const char END_MESSAGE[] = "\r\n";
const struct device *spi;
const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
static const struct gpio_dt_spec spi_activate_pin = GPIO_DT_SPEC_GET(spi_act_pin, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct device *gsm_dev;
static struct net_mgmt_event_callback mgmt_cb;
static bool starting = IS_ENABLED(CONFIG_GSM_PPP_AUTOSTART);

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, 32, 10, 4);

typedef struct //Measurment
parameters
{
	float dummy;
	
	float voltage_ab;
	float voltage_bc;
	float voltage_ca;
	
	float voltage_a;
	float voltage_b;
	float voltage_c;
	float voltage_avg;

	float current_a;
	float current_b;
	float current_c;
	float current_n;
	float current_avg;
	
	float active_power_a;
	float active_power_b;
	float active_power_c;
	float total_active_power;
	
	float apr_power_a;
	float apr_power_b;
	float apr_power_c;
	float total_apr_power;	
	
	float reactive_power_a;
	float reactive_power_b;
	float reactive_power_c;
	float total_reactive_power;
	
	float power_factor_a;
	float power_factor_b;
	float power_factor_c;
	float total_power_factor;
	float frequency;
	
	uint8_t input[NUM_OF_INPUTS];
	uint8_t inputH2L[NUM_OF_INPUTS];
	uint8_t inputL2H[NUM_OF_INPUTS];
	uint8_t output[NUM_OF_OUTPUTS];
	
	float angle0;
	float angle1;
	float angle2;
	
	float voltage_thd;
	float current_thd;	
	
	uint8_t alarm;
	
	uint8_t energy_reset;
	uint8_t alarm_reset;
		
}Measurment_t;

struct spi_cs_control spi_cs = {
	.gpio = GPIO_DT_SPEC_GET(SPI_CS, gpios),
	.delay = 10
};

struct spi_config spi_cfg = {
	.frequency = 1000000,
	.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_FULL_DUPLEX | SPI_MODE_GET(0) | SPI_TRANSFER_MSB,
	.slave = 1, 
	.cs = &spi_cs
};

struct spi_buf rx_bufs[] = { { .buf = rx_buf_spi, .len = SPI_BUF_SIZE } };

struct spi_buf_set spi_rx_buf = { .buffers = rx_bufs, .count = 1 };

static int cmd_sample_modem_suspend(const struct shell *shell, size_t argc, char *argv[])
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!starting) {
		shell_fprintf(shell, SHELL_NORMAL, "Modem is already stopped.\n");
		return -ENOEXEC;
	}

	gsm_ppp_stop(gsm_dev);
	starting = false;

	return 0;
}

static int cmd_sample_modem_resume(const struct shell *shell, size_t argc, char *argv[])
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (starting) {
		shell_fprintf(shell, SHELL_NORMAL, "Modem is already started.\n");
		return -ENOEXEC;
	}

	gsm_ppp_start(gsm_dev);
	starting = true;

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sample_commands, SHELL_CMD(resume, NULL, "Resume the modem\n", cmd_sample_modem_resume),
	SHELL_CMD(suspend, NULL, "Suspend the modem\n", cmd_sample_modem_suspend),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sample, &sample_commands, "Sample application commands", NULL);

static void event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
			  struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if ((mgmt_event & (NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED)) != mgmt_event) {
		return;
	}

	if (mgmt_event == NET_EVENT_L4_CONNECTED) {
		LOG_INF("Network connected");
		return;
	}

	if (mgmt_event == NET_EVENT_L4_DISCONNECTED) {
		LOG_INF("Network disconnected");
		return;
	}
}

static void modem_on_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	LOG_INF("GSM modem on callback fired");
}

static void modem_off_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	LOG_INF("GSM modem off callback fired");
}

void gsm_init()
{
	gsm_dev = DEVICE_DT_GET(DT_INST(0, zephyr_gsm_ppp));

	/* Optional register modem power callbacks */
	gsm_ppp_register_modem_power_callback(gsm_dev, modem_on_cb, modem_off_cb, NULL);

	LOG_INF("Board '%s' APN '%s' UART '%s' device %p (%s)", CONFIG_BOARD, CONFIG_MODEM_GSM_APN,
		uart_dev->name, uart_dev, gsm_dev->name);

	net_mgmt_init_event_callback(&mgmt_cb, event_handler,
				     NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED);
	net_mgmt_add_event_callback(&mgmt_cb);
}

void mdm_cmd_recv_cb(const struct device *uart_dev)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {

		uart_fifo_read(uart_dev, &c, 1);
	
		if (rx_buf_uart_pos < (sizeof(rx_buf_uart) - 1))
			rx_buf_uart[rx_buf_uart_pos++] = c;
		/* else: characters beyond buffer size are dropped */
	}
	
	k_msgq_put(&uart_msgq, &rx_buf_uart, K_NO_WAIT);

	return;

}

void send_modem(uint8_t *command)
{
	size_t size_of_command = strlen(command);
	
	for (int i = 0; i < size_of_command; i++)
		uart_poll_out(uart_dev, command[i]);
}

void receive_response()
{
	k_msgq_get(&uart_msgq, &rx_buf_uart, K_FOREVER);
	k_msleep(RESPONSE_DELAY);
	rx_buf_uart_pos = 0;
	for(int i = 0; i < sizeof(rx_buf_uart); i++)
		rx_buf_uart[i] = 0;
}

void tcp_init(char *server_address, char *server_port)
{
        //Reset Modem to avoid conflict:
	send_modem("AT+CFUN=1,0");
	send_modem(END_MESSAGE);
	receive_response();
		
	send_modem("AT&F0");
	send_modem(END_MESSAGE);
	receive_response();

	send_modem("ATE0");
	send_modem(END_MESSAGE);
	receive_response();	

	send_modem("AT+QICLOSE=0");
	send_modem(END_MESSAGE);
	receive_response();
	
	send_modem("AT+QIDEACT=1");
	send_modem(END_MESSAGE);
	receive_response();
	
	send_modem("AT+QICSGP=1,1,\"mtnirancell\",\"\",\"\",0");
	send_modem(END_MESSAGE);
	receive_response();	

	send_modem("AT+QIACT=1");
	send_modem(END_MESSAGE);
	receive_response();
			
	send_modem("AT+QIOPEN=1,0,\"TCP\",\"");	
	send_modem(server_address);
	send_modem("\",");
	send_modem(server_port);
	send_modem(",0,0");			
	send_modem(END_MESSAGE);
	receive_response();	

	send_modem("AT+QURCCFG=\"urcport\",\"uart1\"");
	send_modem(END_MESSAGE);
	receive_response();
	
	send_modem("AT+QISWTMD=0,0");
	send_modem(END_MESSAGE);
	receive_response();
}

void tcp_send(char *send_massage)
{
	send_modem("AT+QISEND=0");
	send_modem(END_MESSAGE);
	receive_response();	
	
	char ctrl_Z = 26; 
	send_modem(send_massage);
	send_modem("\n");
	send_modem(&ctrl_Z);
	send_modem(END_MESSAGE);
	receive_response();
}

void tcp_receive()
{
	//1500 is max length of receiving data
	send_modem("AT+QIRD=0,1500");
	send_modem(END_MESSAGE);
	receive_response();
}

void uart_init()
{
	uart_irq_callback_user_data_set(uart_dev, mdm_cmd_recv_cb, NULL);
	uart_irq_rx_enable(uart_dev);	
}

void spi_init()
{
	spi = DEVICE_DT_GET(SPI_NODE); 
}

void spi_activate()
{
	gpio_pin_configure_dt(&spi_activate_pin, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&spi_activate_pin, GPIO_ACTIVE_HIGH);
}
int a;
int main(void)
{
		
//starting uart communication:
	uart_init();
	
//starting spi communication between modem's MCU and main MCU:
	spi_init();
	spi_activate();
	
//starting gsm-modem:
	gsm_init();

	//spi_irq_callback_user_data_set(spi, spi_receive_cb, NULL);
	a = gpio_pin_get_dt(&spi_activate_pin);
	
	//uart_irq_rx_enable(spi);
	//spi_receive();	
	
	int ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
		
	//tcp_init("postman-echo.com", "80");			
	
	//tcp_init("tcpbin.com", "4242");
	
	//tcp_init("smtp.gmail.com", "465");
	
	tcp_init("google.com", "80");
	
	tcp_send("test1");	
	tcp_receive();
	tcp_receive();
	
	tcp_init("google.com", "80");
	
	tcp_send("test2");	
	tcp_receive();
	tcp_receive();
	
	tcp_init("google.com", "80");
	
	tcp_send("test3");	
	tcp_receive();
	tcp_receive();
	
	tcp_init("google.com", "80");
	
	tcp_send("test4");	
	tcp_receive();
	tcp_receive();
	
	tcp_init("google.com", "80");
	
	tcp_send("test5");	
	tcp_receive();
	tcp_receive();
									
/*	mdm_cmd_send(uart_dev, "AT+QMTCFG=\"aliauth\",1\r\n", strlen("AT+QMTCFG=\"aliauth\",1\r\n"));
	mdm_cmd_recv();
	
	mdm_cmd_send(uart_dev, "AT+QMTCFG=\"version\",0,4\r\n", strlen("AT+QMTCFG=\"version\",0,4\r\n"));
	mdm_cmd_recv();	
	
	mdm_cmd_send(uart_dev, "AT+QMTCFG=\"recv/mode\",0,0,1\r\n", strlen("AT+QMTCFG=\"recv/mode\",0,0,1\r\n"));
	mdm_cmd_recv();	
	
	mdm_cmd_send(uart_dev, "AT+QMTOPEN=0,\"broker.hivemq.com\",1883\r\n", strlen("AT+QMTOPEN=0,\"broker.hivemq.com\",1883\r\n"));
	mdm_cmd_recv();	

	mdm_cmd_send(uart_dev, "AT+QMTCONN=0,\"1947\"\r\n", strlen("AT+QMTCONN=0,\"1947\"\r\n"));
	mdm_cmd_recv();			
	
/*
	ARG_UNUSED(gsm_dev);
	gsm_ppp_stop(gsm_dev);
*/
	Measurment_t measurment;
	while (true) {
/*		ARG_UNUSED(gsm_dev);
		gsm_ppp_start(gsm_dev);
*/		ret = gpio_pin_toggle_dt(&led1);
		k_msleep(SLEEP_TIME_MS);
		a = spi_transceive(spi, &spi_cfg, NULL, &spi_rx_buf);
	        crc = ~(crc_32(rx_buf_spi, 1020));		
		memcpy(&measurment, &rx_buf_spi, sizeof(Measurment_t));
/*		ARG_UNUSED(gsm_dev);
		gsm_ppp_stop(gsm_dev);
*/	}
	return 0;
}
