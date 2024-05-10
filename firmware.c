 //Bibliotecas utilizadas:
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "soc/clk_tree_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define POT ADC_CHANNEL_0		//Canal do ADC do potenciômetro
#define COOLER GPIO_NUM_25		//GPIO do PWM

#define BUFF_LEN 10			//tamanho do buffer de envio

#define SKIP_SAMPLES 0			//pula 'SKIP_SAMPLES' amostras para salvar no buffer (0 significa que irá inviar tantas amostras quanto há na amostagem original)

#define ts 1					//tempo de amostragem, em ms

#define FILTER_SAMPLES 10		//número de amostras do filtro de média móvel

#define CONFIG_ESP_INT_WDT_TIMEOUT_MS 10000		//Configuração do timeout do watchdog
#define CONFIG_ESP_TASK_WDT_TIMEOUT_S 2000		//Configuração do timeout de tasks
#define CONFIG_ESP_TIMER_INTERRUPT_LEVEL 2		//Nível de prioridade da interrupção

#define PARALLEL_DATAS 3	//Número de sinais a serem enviados paralelalemnte

#define SIG_STEP 5			//Passo do sinal entre cada ponto da curva, de 0 a 100% (a cada SIG_STEP o angulo é medido e registrado)

#define N_REG 300			//Número de pontos para a regressão da reta tangente (análise da resposta ao degrau)

#define TP_DELAY 125		//Atraso de transporte estimado, em milissegundos
#define TAU 50				//Constante de tempo do sistema, em millisegundos

#define MIN_ANGLE 0		//Ângulo mínimo de operação do plate (0 graus)
#define MAX_ANGLE 100		//Ângulo máximo do plate (limite mecânico)

#define TAMA 2				//Tempo de Acomodação em Malha Aberta (superdimensionado, aproximado, em segundos)

#define AMP_DEG 20			//Amplitude do degrau de teste, em graus (parâmetros por step response)

//Declaração e inicialização de variáveis:
float ref=0, filtered=0, output=0, usig=0, kp=0, ki=0, kd=0, sg=1, offset=0, e=0, ea=0, ie=0, de=0, pop=0, current_time=0, d0=0, df=0, t0=0, mx=0, my=0, som=0;
int mode=0, valpot=0, counter=0, filter_count=0, samples_count=0, reg_count=0, cpa=0;
long accum_prbs=0, cpd=0; //current prbs duration
short id=0;
char topic[15], data[15];
adc_oneshot_unit_handle_t handle = NULL;
esp_mqtt_client_handle_t client = NULL;
float curve[((int)100/SIG_STEP)+1][2], samples[((int)TAU/(10*ts))-1]; //curve -> sinal x ângulo
float x[N_REG], y[N_REG]; //Vetores para a regressão
bool analyze=false, start=false, samp=false;

//Struct de dados para argumentos da freeRTOS task:
typedef struct
{
    float buff1[PARALLEL_DATAS][BUFF_LEN];
    float buff2[PARALLEL_DATAS][BUFF_LEN];
    bool b1f;
    bool b2f;
} Buffer;
Buffer buff;

//Função para envio de buffer via MQTT:
void send_data(float buffer[PARALLEL_DATAS][BUFF_LEN])
{
	char aux[12], send[12*PARALLEL_DATAS*BUFF_LEN];
	send[0] = '\0';
	for(int i=0; i<PARALLEL_DATAS; i++)
	{
		for(int j=0; j<BUFF_LEN; j++)
		{
			aux[0] = '\0';
			sprintf(aux, "%.3f", buffer[i][j]);
			strcat(send, aux);
			if(j<(BUFF_LEN-1))
			{
				strcat(send, ",");
			}
		}
		if(i<(PARALLEL_DATAS-1))
		{
			strcat(send, "*");
		}
	}
	esp_mqtt_client_publish(client, "plot", send, 0, 1, 0);
}

//Função de saturação para o sinal de PWM:
float sat(float val)
{
	if(val>100)
	{
		return 100;
	}
	if(val<0)
	{
		return 0;
	}
	return val;
}

//Função para converter os bits do ADC em graus:
float convert(int analog)
{
	if(offset)
	{
		return ((float)((float)(analog-offset)*(((float)MAX_ANGLE-MIN_ANGLE)/(float)sg))+(float)MIN_ANGLE);
	}
	return ((float)analog);
}

//Função para limpar o buffer:
void clr_buff()
{
	for(int i=0; i<BUFF_LEN; i++)
	{
		for(int j=0; j<PARALLEL_DATAS; j++)
		{
			buff.buff1[j][i] = 0;
			buff.buff2[j][i] = 0;
		}
	}
	id = 0;
	counter = 0;
	buff.b1f = false;
	buff.b2f = false;
}

//Função para atualizar o filtro:
void get_sample()
{
	adc_oneshot_read(handle, POT, &valpot);
	som += convert(valpot);
	filter_count++;
}

//Função para atualizar a amostragem do ângulo:
void update_sample()
{
	filtered = ((float)som/(float)FILTER_SAMPLES);
	filter_count = 0;
	som = 0;
}

//Função para configurar o dutty cycle:
void set_duty(float duty)
{
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ((int)(8191*duty/100.0)));
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

//Função para retorno do sinal em função do ângulo:
float u_calc(float ang)
{
	float delta=100;
	int closest=0;
	for(int i=0; i<(((int)100/SIG_STEP)+1); i++)
	{
		if(fabs(ang-curve[i][1])<delta)
		{
			delta = fabs(ang-curve[i][1]);
			closest = i;
		}
	}
	if(curve[closest][1])
	{
		return ((float)(curve[closest][0]*ang)/(float)curve[closest][1]);
	}else
	{
		return 0;
	}
}

//Função para calcular o sinal PRBS naquele instante:
int getPRBS()
{
	accum_prbs += ts;
	int rndval = 0;
	if(accum_prbs > cpd)
	{
		do
		{
			rndval = ((TAU/10)+(random()%((7*TAU)/30)));
		}while(rndval == cpd);
		cpd = rndval;
		accum_prbs = 0;
		do
		{
			rndval = ((random()%3)-1);
		}while(rndval == cpa);
		cpa = rndval;
	}
	return cpa;
}

//Função para a malha aberta:
void open_loop()
{
	//Para degraus em malha aberta:
	usig = sat(u_calc(ref));
	//Para PRBS em malha aberta:
	//usig = sat(u_calc(pop)+(getPRBS()*ref));
	set_duty(usig);
}

//Função para a malha fechada:
void closed_loop()
{
	//Calcular o sinal de controle:
	ea = e;
	e = (ref-output);
	ie += (e*ts*1e-3);
	//Satura a ação integral:
	/*if(fabs(ie)>50)
	{
		ie = (((float)fabs(ie)/(float)ie)*50.0);
	}*/
	de = ((float)(e-ea)/(float)(ts*1e-3));
	usig = sat(u_calc(pop)+((kp*e) + (ki*ie) + (kd*de)));

	//Aplicar PWM:
	set_duty(usig);
}

//Função para atualizar o buffer:
void att_buff()
{
	if(counter == SKIP_SAMPLES)
	{
		if(id<BUFF_LEN)
		{
			buff.buff1[0][id] = output;
			buff.buff1[1][id] = ref;
			buff.buff1[2][id] = usig;
			id++;
			if(id == BUFF_LEN)
			{
				buff.b1f = true;
			}
		}else
		{
			buff.buff2[0][id%10] = output;
			buff.buff2[1][id%10] = ref;
			buff.buff2[2][id%10] = usig;
			id++;
			if(id == (2*BUFF_LEN))
			{
				buff.b2f = true;
				id = 0;
			}
		}
	}
	counter++;
	if(counter>SKIP_SAMPLES)
	{
		counter = 0;
	}
}

//Função para pegar os parâmetros do transitório:
void transient_analysis()
{
	//Ponto de operação:
	if(current_time<TAMA)
	{
		usig = sat(u_calc(ref));
		set_duty(usig);
		t0 = current_time;
		d0 = output;
	}else
	{
		//Aplica o degrau:
		if(current_time < (2*TAMA))
		{
			ref = (pop+AMP_DEG);
			usig = sat(u_calc(pop+AMP_DEG));
			set_duty(usig);
			if(current_time > (TP_DELAY*1e-3))
			{
				if(reg_count < N_REG)
				{
					mx += (current_time-t0);
					my += output;
					x[reg_count] = (current_time-t0);
					y[reg_count] = output;
					reg_count++;
				}
				df = output;
			}
		}else //Retorna o plate:
		{
			ref = 0;
			usig = sat(u_calc(ref));
			set_duty(usig);
		}
	}
	current_time += (ts*1e-3);
}

//Função de callback para o timer da malha de controle:
static void periodic_timer_callback(void* arg)
{
	if(mode)
	{
		get_sample();
	}else
	{
		clr_buff();
		ie = 0;
		som = 0;
		filter_count = 0;
		samples_count = 0;
		samp = false;
	}
	if(filter_count == FILTER_SAMPLES)
	{
		if(mode)
		{
			update_sample();
		}
		if(samp)
		{
			//Atualização do filtro dinâmico:
			float accum=0;
			for(int i=0; i<(TAU/(10*ts))-1; i++)
			{
				accum += (float)samples[i]*((10*ts)/(float)TAU);
				if(i>0)
				{
					samples[i-1] = samples[i];
				}
			}
			samples[(TAU/(10*ts))-2] = filtered;
			accum += (float)filtered*((10*ts)/(float)TAU);
			output = accum;
			if(analyze)
			{
				transient_analysis();
			}
			if(start)
			{
				if(mode==2)
				{
					open_loop();
				}
				if(mode==3)
				{
					closed_loop();
				}
			}
			if(mode)
			{
				att_buff();
			}
		}else
		{
			//Preenche as posições anteriores do filtro:
			samples[samples_count] = filtered;
			samples_count++;
			if(samples_count == (TAU/(10*ts))-1)
			{
				samp = true;
			}
		}
	}
}

//Função de callback para o MQTT:
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	esp_mqtt_event_handle_t event = event_data;
	sprintf(topic, "%.*s", event->topic_len, event->topic);
	sprintf(data, "%.*s", event->data_len, event->data);
	if(!strcmp(topic, "mode"))
	{
		mode = atoi(data);
	}
	if(!strcmp(topic, "controller"))
	{
		int i=0, pos=0, mark=0;
		float consts[3];
		char cut[12];
		for(i=0; i<event->data_len; i++)
		{
			if(data[i] != ',')
			{
				cut[i-mark] = data[i];
			}else
			{
				cut[i-mark] = '\0';
				consts[pos] = atof(cut);
				pos++;
				mark = i+1;
				cut[0] = '\0';
			}
		}
		cut[i-mark+1] = '\0';
		consts[pos] = atof(cut);
		kp = consts[0];
		ki = consts[1];
		kd = consts[2];
	}
	if(!strcmp(topic, "reference"))
	{
		ref = atof(data);
	}
}

//Função para inicializar o wifi:
void wifi()
{
	//Inicializando wifi:
	nvs_flash_init();
	esp_netif_init();
	esp_event_loop_create_default();
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&wifi_initiation);
	wifi_config_t wifi_configuration = {
		.sta = {
					.ssid 		= "Erick",
					.password 	= "fanplate",
		},
	};
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
	esp_wifi_start();
	esp_wifi_connect();
}

//Função para inicializar e conectar ao broker MQTT:
void mqtt()
{
	//Configuração do MQTT:
	esp_event_loop_create_default();
	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = "mqtt://192.168.137.1:1884/mqtt",
		.credentials.client_id	= "ESP32",
	};
	client = esp_mqtt_client_init(&mqtt_cfg);

	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

	esp_mqtt_client_start(client);

	//Subscrevendo nos tópicos:
	esp_mqtt_client_subscribe(client, "mode", 1);
	esp_mqtt_client_subscribe(client, "controller", 1);
	esp_mqtt_client_subscribe(client, "reference", 1);
}

//Função para configurar o ADC:
void cfg_ADC()
{
	//Configuração do ADC:
	adc_oneshot_unit_init_cfg_t init_cfg = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	adc_oneshot_new_unit(&init_cfg, &handle);

	adc_oneshot_chan_cfg_t ch_cfg = {
		.bitwidth = ADC_BITWIDTH_12,
		.atten = ADC_ATTEN_DB_6,
	};
	adc_oneshot_config_channel(handle, POT, &ch_cfg);
}

//Função para configurar o timer da malha de controle:
void cfg_periodic_timer()
{
	esp_timer_handle_t periodic_timer = NULL;
	const esp_timer_create_args_t periodic_timer_args = {
		.callback = &periodic_timer_callback,
		.name = "control"
	};
	esp_timer_create(&periodic_timer_args, &periodic_timer);
	esp_timer_start_periodic(periodic_timer, (ts*1e3/FILTER_SAMPLES));
}

//Função para configurar o PWM:
void cfg_PWM()
{
	//Inicialização do PWM:
	ledc_timer_config_t pwm1c = {
		.speed_mode 		= LEDC_LOW_SPEED_MODE,
		.timer_num 			= LEDC_TIMER_0,
		.duty_resolution 	= LEDC_TIMER_13_BIT,
		.freq_hz 			= 1000,
		.clk_cfg 			= LEDC_AUTO_CLK
	};
	ledc_timer_config(&pwm1c);
	ledc_channel_config_t pwm1ch = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel 	= LEDC_CHANNEL_0,
		.timer_sel 	= LEDC_TIMER_0,
		.intr_type 	= LEDC_INTR_DISABLE,
		.gpio_num 	= COOLER,
		.duty		= 0,
		.hpoint		= 0,
	};
	ledc_channel_config(&pwm1ch);
}

//Rotina de calibração do sensor:
void sensor_calibration()
{
	unsigned long min=0;
	for(int i=0; i<1000; i++)
	{
		min += valpot;
	}
	//Definição do limite angular máximo:
	usig = 100;
	set_duty(usig);
	sleep(TAMA);
	unsigned long max=0;
	for(int i=0; i<1000; i++)
	{
		max += valpot;
	}

	//Definição das constantes de ajuste do sinal:
	offset = ((float)min/1000.0);
	sg = ((float)((max-min)/1000.0));
}

//Rotina para obtenção da curva de sinal x ângulo:
void get_actuation_curve()
{
	float med_val;
	for(float i=0; i<=100; i+=SIG_STEP)
	{
		usig = i;
		set_duty(usig);
		sleep(TAMA); //aguardar acomodação
		med_val = 0;
		for(int j=0; j<50; j++)
		{
			med_val += output;
		}
		curve[(int)i/SIG_STEP][0] = i;
		curve[(int)i/SIG_STEP][1] = ((float)med_val/50.0);
	}
}

//Rotina para obter o modelo do sistema e projeto dos controladores:
void get_models()
{
	float s1=0, s2=0, k=0, theta=0, tau=0, C=0;

	//Regressão Linear:
	mx /= N_REG;
	my /= N_REG;
	for(int i=0; i<N_REG; i++)
	{
		s1 += (x[i]*(my-y[i]));
		s2 += (x[i]*(mx-x[i]));
	}
	float a = ((float)s1/(float)s2);
	float b = (my - (a*mx));

	printf("\n Reta tangente: y = %gx + %g", a, b);

	printf("\n ! Modelo do sistema obtido !\n");
	//Obtendo modelo do sistema:
	if(u_calc(AMP_DEG))
	{
		k = ((float)(df-d0)/(float)u_calc(AMP_DEG));
		printf("\n k = %g\n", k);
	}else
	{
		printf("\n * Não foi possível obter o ganho estático *");
	}
	if(a)
	{
		theta = ((float)-b/(float)a);
		printf("\n theta = %g\n", theta);
		C = ((float)(df-b)/(float)a);
		tau = (C-theta);
		printf("\n tau = %g\n", tau);
	}else
	{
		printf("\n * Não foi possível obter o atraso de trasporte e a constante de tempo *");
	}

	//Obtendo controlador por Ziegler-Nichols:
	/*printf("\n\n\n ==> Controlador PI Ziegler-Nichols <== \n");
	if(theta&&m)
	{
		kp = ((float)0.9/(float)(theta*m));
		printf("\n kp = %g", kp);
	}else
	{
		printf("\n * Não foi possível obter kp *");
	}
	if(theta)
	{
		ki = ((float)3.0/(float)(10.0*theta));
		printf("\n ki = %g", ki);
	}else
	{
		printf("\n * Não foi possível obter ki *");
	}*/

	//Obtendo controlador por CHR:
	printf("\n\n\n ==> Controlador PI CHR <== \n");
	if(k&&theta)
	{
		kp = fabs((float)(0.6*tau)/(float)(k*theta));
		printf("\n kp = %g\n", kp);
	}else
	{
		printf("\n * Não foi possível obter kp *");
	}
	if(theta)
	{
		ki = fabs((float)1.0/(float)(4.0*theta));
		printf("\n ki = %g\n", ki);
	}else
	{
		printf("\n * Não foi possível obter ki *");
	}


	//Obtendo controlador por Cohen:


	//Enviar parâmetros do modelo e controlador:
	char aux[12], send[6*12];
	send[0] = '\0';
	//Ganho:
	aux[0] = '\0';
	sprintf(aux, "P%.4f,", k);
	strcat(send, aux);
	//Atraso de transporte:
	aux[0] = '\0';
	sprintf(aux, "%.4f,", theta);
	strcat(send, aux);
	//Constante de tempo:
	aux[0] = '\0';
	sprintf(aux, "%.4f,", tau);
	strcat(send, aux);
	//kp:
	aux[0] = '\0';
	sprintf(aux, "%.4f,", kp);
	strcat(send, aux);
	//ki:
	aux[0] = '\0';
	sprintf(aux, "%.4f,", ki);
	strcat(send, aux);
	//kd:
	aux[0] = '\0';
	sprintf(aux, "%.4f", kd);
	strcat(send, aux);
	//Publica no tópico:
	esp_mqtt_client_publish(client, "plot", send, 0, 1, 0);
}

//Função para looping de envio de dados via MQTT:
void sending_loop(void *args)
{
	Buffer *bf = args;
	//Loop de envio dos dados ao front-end:
	while(1)
	{
		if(bf->b1f)
		{
			send_data(bf->buff1);
			bf->b1f = false;
		}
		if(bf->b2f)
		{
			send_data(bf->buff2);
			bf->b2f = false;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

//Função principal:
void app_main()
{
	//Definindo a semente do gerador aleatório:
	srand(time(NULL));

	//Inicializando a matriz curve:
	for(int i=0; i<(((int)100/SIG_STEP)+1); i++)
	{
		for(int j=0; j<2; j++)
		{
			curve[i][j] = 0;
		}
	}

	//Configuração das GPIO's:
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);

	//Inicializações necessárias:
	wifi();
	printf("\n! Wifi configurado!\n");
	sleep(3);
	mqtt();
	printf("\n! MQTT configurado!\n");
	sleep(1);
	cfg_ADC();
	printf("\n! ADC configurado!\n");
	sleep(1);
	cfg_periodic_timer();
	printf("\n! Timer configurado!\n");
	cfg_PWM();
	printf("\n! PWM configurado!\n");
	xTaskCreate(
			&sending_loop,
			"sending_loop",
			8192,
			&buff,
			1,
			NULL
	);

	esp_mqtt_client_publish(client, "feedback", "F0", 0, 1, 0);
	//Rotinas de calibração:
	while(!mode)
	{
		gpio_set_level(GPIO_NUM_2, 1);
		printf("\n Aguardando comando de calibração... \n");
		usleep(500000);
		gpio_set_level(GPIO_NUM_2, 0);
		usleep(500000);
	}
	gpio_set_level(GPIO_NUM_2, 0);
	printf("\n Procedimento de calibração iniciado...\n");
	sensor_calibration();
	ref = 0;
	usig = 0;
	set_duty(usig);
	sleep(TAMA);
	printf("\n ! Sensor calibrado !\n");
	esp_mqtt_client_publish(client, "feedback", "F1", 0, 1, 0);
	get_actuation_curve();
	printf("\n ! Curva de atuação adquirida !\n");
	ref = 0;
	usig = 0;
	set_duty(usig);
	sleep(TAMA);
	esp_mqtt_client_publish(client, "mode", "0", 0, 1, 0);
	esp_mqtt_client_publish(client, "feedback", "F2", 0, 1, 0);
	while(!ref)
	{
		printf("\n Aguardando POP...\n");
	}
	esp_mqtt_client_publish(client, "feedback", "F3", 0, 1, 0);
	pop = ref;
	esp_mqtt_client_publish(client, "mode", "1", 0, 1, 0);
	//analyze = true;
	//sleep(3*TAMA);
	//analyze = false;
	esp_mqtt_client_publish(client, "mode", "0", 0, 1, 0);
	//printf("\n ! Análise da resposta ao degrau concluída !\n");
	esp_mqtt_client_publish(client, "feedback", "F4", 0, 1, 0);
	//get_models();
	esp_mqtt_client_publish(client, "feedback", "F5", 0, 1, 0);
	kp = 2.8;
	ki = 1.5;
	kd = 0;
	//Aciona o LED BUILT-IN indicando sucesso das configurações, conexões iniciais e calibração:
	gpio_set_level(GPIO_NUM_2, 1);
	printf("\n\n! Sistema pronto para inciar !\n");

	while(1)
	{
		while((mode!=2)&&(mode!=3))
		{
			gpio_set_level(GPIO_NUM_2, 1);
			printf("\n Aguardando comando de início... \n");
			usleep(500000);
			gpio_set_level(GPIO_NUM_2, 0);
			usleep(500000);
		}

		if((mode==2)||(mode==3))
		{
			//Leva sistema ao ponto de operação:
			ref = pop;
			usig = sat(u_calc(ref));
			set_duty(usig);
			esp_mqtt_client_publish(client, "feedback", "F6", 0, 1, 0);
			printf("\n Levando sistema ao ponto de operação...\n");
			sleep(TAMA);

			//Habilita o sistema:
			start = true;
			esp_mqtt_client_publish(client, "feedback", "F7", 0, 1, 0);
		}

		while(mode)
		{
			printf("\n\n! Sistema em operação !\n");
			sleep(1);
		}
		start = false;
		ref = 0;
		usig = 0;
		set_duty(usig);
	}
}
