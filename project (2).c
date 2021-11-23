#include<stdbool.h>	//boolean
#include<stdio.h>	//printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
#include <unistd.h>
#include "pca9685/pca9685.h"
#include <wiringPi.h>
#include <signal.h>
#include <pthread.h>
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50
void udp_receiver( void );
void udp_handler( void );

int fd;
#define BUFLEN 512	//Max length of buffer
#define PORT 8888	//The port on which to listen for incoming data
/*
 * wiringPi C library use different GPIO pin number system from BCM pin numberwhich are often used by Python, 
 * you can lookup BCM/wPi/Physical pin relation by following Linux command : gpio readall
 */
 //define L298N control pins in wPi system
#define ENA 0  //left motor speed pin ENA connect to PCA9685 port 0
#define ENB 1  //right motor speed pin ENB connect to PCA9685 port 1
#define IN1 4  //Left motor IN1 connect to wPi pin# 4 (Physical 16,BCM GPIO 23)
#define IN2 5  //Left motor IN2 connect to wPi pin# 5 (Physical 18,BCM GPIO 24)
#define IN3 2  //right motor IN3 connect to wPi pin# 2 (Physical 13,BCM GPIO 27)
#define IN4 3  //right motor IN4 connect to wPi pin# 3 (Physical 15,BCM GPIO 22)

//define IR tracking sensor wPi pin#
#define sensor1 21 // No.1 sensor from far left to wPi#21 Physical pin#29
#define sensor2 22 // No.2 sensor from left to wPi#22 Physical pin#31
#define sensor3 23 // middle sensor to wPi#23 Physical pin#33
#define sensor4 24 // No.2 sensor from right to wPi#24 Physical pin#35
#define sensor5 25 // No.1 sensor from far  right to wPi#25 Physical pin#37
char val[5]; //sensor value array


#define SERVO_PIN 15  //right motor speed pin ENB connect to PCA9685 port 1
#define LEFT 400 //ultrasonic sensor facing right
#define CENTER 280//ultrasonic sensor facing front
#define RIGHT 160 //ultrasonic sensor facing left
#define TRIG 28 //wPi#28=BCM GPIO#20=Physical pin#38
#define ECHO 29 //wPi#29=BCM GPIO#21=Physical pin#40
#define OBSTACLE 20
#define OBJ_DISTANCE 8
#define short_delay 200
#define long_delay  300
#define extra_long_delay 400

int sts1=0;
int sts2=0;
int sts3=0;
char o_val[3];

#define high_speed 3000  // Max pulse length out of 4096
#define SPEED  2000  // Max pulse length out of 4096
#define low_speed  1350  // Max pulse length out of 4096
#define LOW_SPEED 1300
#define track_short_delay 100
#define track_long_delay 200
#define track_extra_track_long_delay 300
char cur_status='E';

//track previous status
char pre_status= 'E';

//track status of obstacle ative
//first case set manual on
bool obst_status = true;

void setup(){
 pinMode(IN1,OUTPUT);
 pinMode(IN2,OUTPUT);
 pinMode(IN3,OUTPUT);
 pinMode(IN4,OUTPUT);
 pinMode(sensor1,INPUT);
 pinMode(sensor2,INPUT);
 pinMode(sensor3,INPUT);
 pinMode(sensor4,INPUT);
 pinMode(sensor5,INPUT);
 pinMode(TRIG,OUTPUT);
 pinMode(ECHO,INPUT); 
 
 digitalWrite(IN1,LOW);
 digitalWrite(IN2,LOW);
 digitalWrite(IN3,LOW);
 digitalWrite(IN4,LOW);
}
void go_Back(int fd,int l_speed,int r_speed){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW); 
    pca9685PWMWrite(fd, ENA, 0, l_speed);
    pca9685PWMWrite(fd, ENB, 0, r_speed);
}
void go_Advance(int fd,int l_speed,int r_speed){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH); 
    pca9685PWMWrite(fd, ENA, 0, l_speed);
    pca9685PWMWrite(fd, ENB, 0, r_speed);
}
void go_Left(int fd,int l_speed,int r_speed){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH); 
    pca9685PWMWrite(fd, ENA, 0, l_speed);
    pca9685PWMWrite(fd, ENB, 0, r_speed);
}
void go_Right(int fd,int l_speed,int r_speed){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW); 
    pca9685PWMWrite(fd, ENA, 0, l_speed);
    pca9685PWMWrite(fd, ENB, 0, r_speed);
}
void stop_car(int fd){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW); 
    pca9685PWMWrite(fd, ENA, 0, 0);
    pca9685PWMWrite(fd, ENB, 0, 0);
}
// ctrl-C key event handler
void my_handler(int s){
           stop_car( fd);
           printf("Ctrl C detected %d\n",s);
           exit(1); 

}
struct sigaction sigIntHandler;
struct sockaddr_in si_me, si_other;
int s, i, slen = sizeof(si_other) , recv_len;
char buf[BUFLEN];

void die(char *s)
{
	perror(s);
	exit(1);
}
int distance() {
        //Send trig pulse
         //printf("pass1\n");
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG, LOW);
 
        //Wait for echo start
        while(digitalRead(ECHO) == LOW);
 
        //Wait for echo end
        long startTime = micros();
        //printf("pass2\n");
        while(digitalRead(ECHO) == HIGH);
        long travelTime = micros() - startTime;
 
        //Get distance in cm
        int distance = travelTime / 58;
 		if (distance==0) distance=1000;
 		//printf("distance=%d\n",distance);
        return distance;
}

void follow_leader()
{
        
		pca9685PWMWrite(fd, SERVO_PIN, 0, LEFT);
		delay(100);
		if (distance()>OBJ_DISTANCE) sts1=1;
		else sts1=0;
		val[0]='0'+sts1;
		
		pca9685PWMWrite(fd, SERVO_PIN, 0, CENTER);
		delay(100);
		if (distance()>OBJ_DISTANCE) sts2=1;
		else sts2=0;
		val[1]='0'+sts2;
		
		pca9685PWMWrite(fd, SERVO_PIN, 0, RIGHT);
		delay(100);
		if (distance()>OBJ_DISTANCE) sts3=1;
		else sts3=0;
		val[2]='0'+sts3;
		
		if (strcmp("100",val)==0)
		{	
			//go left to approach the object
			printf("100 turn left\n");
			go_Left(fd,0,SPEED);
            delay(short_delay); 
		}
		if (strcmp("001",val)==0)
		{
			//go right to approach the object
			printf("001 turn right\n");
			go_Right(fd,SPEED,0);
            delay(short_delay); 		
		}
		if (strcmp("110",val)==0)
		{
			//go slightly to the left
			printf("110 slight left\n");
			go_Left(fd,0,LOW_SPEED);
            delay(short_delay);  	
		}
		if (strcmp("011",val)==0)
		{
			//go slightly to the right
			printf("011 slight right\n");
			go_Right(fd,LOW_SPEED,0);
            delay(short_delay);  
		}  
		if (strcmp("111",val)==0 || strcmp("101",val)==0 || strcmp("010",val)==0  )
		{
			//go foward until approach the range slowly
			printf("%s go forward\n",val);
			go_Advance(fd, LOW_SPEED, LOW_SPEED);
		}        
		if (strcmp("000",val)==0)
		{
			printf("000 stop\n");
            delay(short_delay); 
            stop_car(fd);
            delay(short_delay);	
		}

}

void obstacle_avoid()
{		
		pca9685PWMWrite(fd, SERVO_PIN, 0, LEFT);
		delay(300);
		if (distance()<OBSTACLE) sts1=1;
		else sts1=0;
		
		o_val[0]='0'+sts1;
		 
		pca9685PWMWrite(fd, SERVO_PIN, 0, CENTER);
		delay(300);
		if (distance()<OBSTACLE) sts2=1;
		else sts2=0;
		o_val[1]='0'+sts2;
		 
		pca9685PWMWrite(fd, SERVO_PIN, 0, RIGHT);
		delay(300);
		if (distance()<OBSTACLE) sts3=1;
		else sts3=0;
		o_val[2]='0'+sts3;
		 
		 
		if (strcmp("100",o_val)==0)
		{	
			printf("100 slight right\n");
			go_Right(fd,SPEED,0);
		//Try if car can continue go straight ahead after turn
			//go_Advance( fd,SPEED,SPEED);
		    delay(long_delay);  
		    stop_car(fd);
		    delay(short_delay);
		}
		if (strcmp("001",o_val)==0)
		{
			printf("100 slight left\n");
			go_Left(fd,0,SPEED); 

			//go_Advance( fd,SPEED,SPEED);
		    delay(long_delay);  
		    stop_car(fd);
		    delay(short_delay);		
		}
		if (strcmp("110",o_val)==0)
		{
			printf("110 sharp right\n");
			go_Right(fd,high_speed,low_speed);
			
			//go_Advance( fd,SPEED,SPEED);
		    delay(long_delay);  
		    stop_car(fd);
		    delay(short_delay);	
		}
		if (strcmp("011",o_val)==0)
		{
			printf("011 sharp left\n");
			go_Left(fd,low_speed,high_speed);

			//go_Advance( fd,SPEED,SPEED); 
		    delay(long_delay);  
		    stop_car(fd);
		    delay(short_delay);	
		}  
		if (strcmp("111",o_val)==0 || strcmp("101",o_val)==0 || strcmp("010",o_val)==0  )
		{
			printf("%s sharp turn back\n",val);
			go_Left(fd,high_speed,high_speed);

			//go_Advance( fd,SPEED,SPEED); 
		    delay(extra_long_delay);  
		    stop_car(fd);
		    delay(short_delay);	
		}        
		if (strcmp("000",o_val)==0)
		{
			printf("000 move forward\n");
			go_Advance(fd,SPEED,SPEED);  //Turn left slightly
		    //delay(long_delay);  
		    //stop_car(fd);
		    //delay(short_delay);	
		} 
	 
}

void tracking()
{		
		//read IR sensor from left to right
		val[0]='0'+!digitalRead(sensor1);
		val[1]='0'+!digitalRead(sensor2);
		val[2]='0'+!digitalRead(sensor3);
		val[3]='0'+!digitalRead(sensor4);
		val[4]='0'+!digitalRead(sensor5);
 
 		//based on IR sensor values to take action. 1 stands for Black and 0 stands for white
        if (strcmp("10000",val)==0 || strcmp("01000",val)==0 || strcmp("11000",val)==0)
         { 
             printf("%s sharp left\n",val);
            //The black line is in the left of the car, need  left turn 
            go_Left(fd,low_speed,low_speed);  //Turn left
            delay(track_short_delay);
            stop_car(fd);
            delay(track_short_delay);
        }
        else if (strcmp("11100",val)==0 || strcmp("10100",val)==0)
        {
             printf("%s slight left\n",val);
            go_Advance(fd,low_speed,high_speed);  //Turn left slightly
            delay(track_long_delay); 
            stop_car(fd);
            delay(track_short_delay);
        }
        else if (strcmp("11110",val)==0 || strcmp("01100",val)==0 ||
         strcmp("10010",val)==0 || strcmp("10110",val)==0 || strcmp("11010",val)==0)
        {
            printf("%s ahead left\n",val);
            go_Advance(fd,low_speed,SPEED);  //go ahead
            delay(track_long_delay);
            stop_car(fd);
            delay(track_short_delay);
        }
        else if (strcmp("01110",val)==0 || strcmp("01010",val)==0 ||
         strcmp("00100",val)==0 || strcmp("10101",val)==0 || 
         strcmp("10011",val)==0 || strcmp("11101",val)==0 || 
         strcmp("10111",val)==0 || strcmp("11011",val)==0 || strcmp("11001",val)==0)
        {
             printf("%s ahead\n",val);
            go_Advance(fd,SPEED,SPEED);  //go ahead
            delay(track_long_delay);
            stop_car(fd);
            delay(track_short_delay);
        }
        else if (strcmp("00110",val)==0 || strcmp("01111",val)==0 ||
         strcmp("01001",val)==0 || strcmp("01011",val)==0 || strcmp("01101",val)==0)
        {
             printf("%s ahead right\n",val);
            go_Advance(fd,SPEED,low_speed);  //Turn left
            delay(track_long_delay);
            stop_car(fd);
            delay(track_short_delay);
        }
        else if (strcmp("00111",val)==0 || strcmp("00101",val)==0)
        {
             printf("%s slight right\n",val);
            go_Advance(fd,high_speed,low_speed);  //Turn right slightly
            delay(track_long_delay);
            stop_car(fd);
            delay(track_short_delay);
        }
        else if (strcmp("00001",val)==0 || strcmp("00010",val)==0 ||
         strcmp("00011",val)==0)
        {
             printf("%s sharp right\n",val);
            go_Right(fd,low_speed,low_speed);  //Turn right sharply
            delay(track_short_delay);
            stop_car(fd);
            delay(track_short_delay);
        }  
         else if (strcmp("00000",val)==0)
         {
              printf("%s roll back\n",val);
            go_Back(fd,low_speed,low_speed);  //Turn right sharply
            delay(track_short_delay);
            stop_car(fd);
            delay(track_short_delay);
         }  
        else if (strcmp("11111",val)==0)
        { printf("%s stop\n",val);
          stop_car(fd);
        }
}
int x=0;
int main(void)
{
    //set up wiringPi GPIO 
    if(wiringPiSetup()==-1){
        printf("setup wiringPi failed!\n");
        printf("please check your setup\n");
        return -1;
    }

	
    //set up GPIO pin mode
	setup();
 
	// Setup PCA9685 with pinbase 300 and i2c location 0x40
	fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
	if (fd < 0)
	{
		printf("Error in setup\n");
		return fd;
	}

   	// following 5 lines define ctrl-C events

   	sigIntHandler.sa_handler = my_handler;
   	sigemptyset(&sigIntHandler.sa_mask);
   	sigIntHandler.sa_flags = 0;
   	sigaction(SIGINT, &sigIntHandler, NULL);

    //following 20 lines set up Socket to receive UDP

	//create a UDP socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		die("socket");
	}
	
	// zero out the structure
	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	
	//bind socket to port
	if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
	{
		die("bind");
	}
	pca9685PWMWrite(fd, SERVO_PIN, 0, LEFT);
	delay(1000);

	pca9685PWMWrite(fd, SERVO_PIN, 0, CENTER);
	delay(1000);

	pca9685PWMWrite(fd, SERVO_PIN, 0, RIGHT);
	delay(1000);
	pca9685PWMWrite(fd, SERVO_PIN, 0, CENTER);
	delay(1000);
	
	printf("Waiting for APP command...");
	//keep listening for data
    pthread_t th1, th2;
    
 	pthread_create ( &th2, NULL, ( void* ) udp_handler, NULL );
    pthread_create ( &th1, NULL, ( void* ) udp_receiver, NULL );
 
	while (1);
	close(s);
	return 0;
}

void udp_handler(void){
	while (1)
	{	 
		//printf("handler: %c\n",cur_status);
    	switch(cur_status)
    	{
            case 'A':
            //printf("move forward\n");
            go_Advance( fd,SPEED,SPEED);
            break;

            case 'B':
            //printf("move backward\n");
            go_Back( fd,SPEED,SPEED);
            break;

            case 'L':
            //printf("turn left\n");
            go_Left( fd,0,SPEED);
            break;

            case 'R':
            //printf("turn right\n");
            go_Right( fd,SPEED,0);
            break;
            
            case 'T':
            //printf("Tracking\n");
            tracking();
            break;

	    case 'H':
	    follow_leader();
	    break;

            case 'O':
	    if (obst_status == false)
            {	
		printf("Obstacle Avoidance!\n");
		obstacle_avoid();
         	break;
	    }
	    else if(obst_status == true)
	    {
		printf("Return to manual\n");
		stop_car(fd);
		break;
	    }
                        
            case 'E':
            //printf("stop\n");
            stop_car( fd);
            break;

	    //Using F1 button for acceleration
	    case 'F':

	    //Nested switch
	    switch(pre_status)
	    {
		case 'A':
		go_Advance( fd,high_speed,high_speed);
            	break;

		case 'B':
		go_Back( fd,high_speed,high_speed);
           	break;

		case 'R':
		go_Right( fd,high_speed,0);
            	break;

		case 'L':
		go_Left( fd,0,high_speed);
		break;		

	    }
	    break;

	    //Using F2 button for deceleration
	    case 'G':

	    //Nested switch
	    switch(pre_status)
	    {
		case 'A':
		go_Advance( fd,low_speed,low_speed);
            	break;

		case 'B':
		go_Back( fd,low_speed,low_speed);
           	break;

		case 'R':
		go_Right( fd,low_speed,0);
            	break;

		case 'L':
		go_Left( fd,0,low_speed);
		break;		

	    }
	    break;
	    
      	}
   }
}
void udp_receiver(void)
{
	while(1)
	{	 
		fflush(stdout);
		
		//try to receive some data, this is a blocking call
		if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
		{
			die("recvfrom()");
		}
		//keep on track for pre_status
		pre_status = cur_status;
		cur_status=buf[0];
		
		//check if case 'O' set on/off manual setting
		if(buf[0] == 'O' && obst_status == true)
		{
		//check if first case happen, set manual off
			obst_status = false;
		}
		else if(buf[0] == 'O' && obst_status == false)
		{
			obst_status = true;
		}	
				
	}
}
//  gcc -pthread -o project project.c pca9685/pca9685.c -lwiringPi 