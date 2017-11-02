#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <kaa.h>
#include <platform/kaa_client.h>
#include <kaa_error.h>
#include <kaa_configuration_manager.h>
#include <kaa_logging.h>
#include <gen/kaa_logging_gen.h>
#include <platform/kaa_client.h>
#include <utilities/kaa_log.h>
#include <platform-impl/common/ext_log_upload_strategies.h>

// #include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <math.h>

#define TEMP 1
#define HUM 2
#define LIGHT 3

static int32_t sample_period;
static time_t  last_sample_time;

extern kaa_error_t ext_unlimited_log_storage_create(void **log_storage_context_p, kaa_logger_t *logger);
float readSerial(int);
/* Retrieves current temperature. */
static float get_temperature_sample(void)
{
    /* For the sake of example, random data is used */
    // return rand() % 10 + 25;
    return readSerial(TEMP);
}

static float get_humidity(void)
{
    /* For the sake of example, random data is used */
    // return rand() % 10 + 25;
    return readSerial(HUM);
}

static float get_luminance(void)
{
    /* For the sake of example, random data is used */
    // return rand() % 10 + 25;
    return readSerial(LIGHT);
}

/* Periodically called by Kaa SDK. */
static void example_callback(void *context)
{
    time_t current_time = time(NULL);

    /* Respect sample period */
    if (difftime(current_time, last_sample_time) >= sample_period) {
        float temperature = get_temperature_sample();
        float humidity = get_humidity();
        float light = get_luminance();

        printf("temperature: %.2f", temperature);
        printf("  Humidity: %.2f", humidity);
        printf("  luminance: %.2f\n", light);
        last_sample_time = current_time;

        kaa_user_log_record_t *log_record = kaa_logging_sensor_data_create();
        log_record->temperature = temperature;
        log_record->humdity = humidity;
        log_record->luminance = light;

        kaa_logging_add_record(kaa_client_get_context(context)->log_collector, log_record, NULL);
    }
}

/* Receives new configuration data. */
static kaa_error_t on_configuration_updated(void *context, const kaa_root_configuration_t *conf)
{
    (void) context;

    printf("Received configuration data. New sample period: %i seconds\n", conf->sample_period);
    sample_period = conf->sample_period;

    return KAA_ERR_NONE;
}

float readSerial(int parameter)
{
    int fd;/*File Descriptor*/

// printf("\n +----------------------------------+");
// printf("\n |        Serial Port Read          |");
// printf("\n +----------------------------------+");

/*------------------------------- Opening the Serial Port -------------------------------*/

/* Change /dev/ttyUSB0 to the one corresponding to your system */

    fd = open("/dev/ttyACM1",O_RDWR | O_NOCTTY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
                           /* O_RDWR   - Read/Write access to serial port       */
                        /* O_NOCTTY - No terminal will control the process   */
                        /* Open in blocking mode,read will wait              */
                            
                                                                    
                            
    // if(fd == -1)						/* Error Checking */
    //        printf("\n  Error! in Opening ttyACM0  ");
    // else
    //        printf("\n  ttyACM0 Opened Successfully ");


/*---------- Setting the Attributes of the serial port using termios structure --------- */

struct termios SerialPortSettings;	/* Create the structure                          */

tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

/* Setting the Baud rate */
cfsetispeed(&SerialPortSettings,B57600); /* Set Read  Speed as 9600                       */
cfsetospeed(&SerialPortSettings,B57600); /* Set Write Speed as 9600                       */

/* 8N1 Mode */
SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 


SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

/* Setting Time outs */
SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


// if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
//     printf("\n  ERROR ! in Setting attributes");
// else
//             printf("\n  BaudRate = 57600 \n  StopBits = 1 \n  Parity   = none");
    
    /*------------------------------- Read data from serial port -----------------------------*/

tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */

char read_buffer[64];   /* Buffer to store the data received              */
int  bytes_read = 0;    /* Number of bytes read by the read() system call */
int i = 0;
int j = 0;
int k = 0;
char data[64];
float sensorArray[10] = {0};

bytes_read = read(fd,&read_buffer,64); /* Read the data                   */
    
j = 0;
k = 0;
for (i=0;i<bytes_read;i++){
    data[j] = read_buffer[i];
    if (data[j] == ','){
        sscanf(data,"%f",&sensorArray[k]);
        k++;        
        j = 0;
    }
    else
        j++;
}
// for(i=0;i<10;i++){
//     printf("%.2f\n",parameters[i]);
// }

close(fd); /* Close the serial port */

if (parameter == TEMP)
    return sensorArray[0];
if (parameter == HUM)
    return sensorArray[1];
if (parameter == LIGHT)
    return sensorArray[2];

}

int main(void)
{
    /* Init random generator used to generate temperature */
    srand(time(NULL));

    /* Prepare Kaa client. */
    kaa_client_t *kaa_client = NULL;
    kaa_error_t error = kaa_client_create(&kaa_client, NULL);
    if (error) {
        return EXIT_FAILURE;
    }

    /* Configure notification manager. */
    kaa_configuration_root_receiver_t receiver = {
        .context = NULL,
        .on_configuration_updated = on_configuration_updated
    };

    error = kaa_configuration_manager_set_root_receiver(
        kaa_client_get_context(kaa_client)->configuration_manager,
        &receiver);

    if (error) {
        return EXIT_FAILURE;
    }

    /* Obtain default configuration shipped within SDK. */
    const kaa_root_configuration_t *dflt = kaa_configuration_manager_get_configuration(
        kaa_client_get_context(kaa_client)->configuration_manager);

    printf("Default sample period: %i seconds\n", dflt->sample_period);

    sample_period = dflt->sample_period;
    
    /* Configure data collection. */
    void *log_storage_context         = NULL;
    void *log_upload_strategy_context = NULL;

    /* The internal memory log storage distributed with Kaa SDK. */
    error = ext_unlimited_log_storage_create(&log_storage_context,
        kaa_client_get_context(kaa_client)->logger);

    if (error) {
        return EXIT_FAILURE;
    }

    /* Create a strategy based on timeout. */
    error = ext_log_upload_strategy_create(
        kaa_client_get_context(kaa_client), &log_upload_strategy_context,
        KAA_LOG_UPLOAD_BY_TIMEOUT_STRATEGY);

    if (error) {
        return EXIT_FAILURE;
    }

    /* Strategy will upload logs every 5 seconds. */
    error = ext_log_upload_strategy_set_upload_timeout(log_upload_strategy_context, 5);

    if (error) {
        return EXIT_FAILURE;
    }

    /* Specify log bucket size constraints. */
    kaa_log_bucket_constraints_t bucket_sizes = {
         .max_bucket_size       = 32,   /* Bucket size in bytes. */
         .max_bucket_log_count  = 2,    /* Maximum log count in one bucket. */
    };

    /* Initialize the log storage and strategy (by default, they are not set). */
    error = kaa_logging_init(kaa_client_get_context(kaa_client)->log_collector,
        log_storage_context, log_upload_strategy_context, &bucket_sizes);

    if (error) {
        return EXIT_FAILURE;
    }
    

    /* Start Kaa SDK's main loop. example_callback is called once per second. */
    error = kaa_client_start(kaa_client, example_callback, kaa_client, 1);

    /* Should get here only after Kaa stop. */
    kaa_client_destroy(kaa_client);
    
    if (error) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
