/*******************************************************************************
#                                                                              #
#      MJPG-streamer allows to stream JPG frames from an input-plugin          #
#      to several output plugins                                               #
#                                                                              #
#      Copyright (C) 2007 Tom St�öveken                                         #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; version 2 of the License.                      #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <syslog.h>

#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/un.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "pxlib.h"
#include "parameter.h"

#include "jpeglib.h"

#include <turbojpeg.h>


#include <linux/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>

#include "mjpg_streamer.h"
#include "utils.h"


#define INPUT_PLUGIN_NAME "PHENOX input plugin"

/* private functions and variables to this plugin */
static pthread_t   worker;
static globals     *pglobal;
static pthread_mutex_t controls_mutex;
static int plugin_number;
static int jpegquality = 70;
void *worker_thread(void *);
void worker_cleanup(void *);
void help(void);

static int delay = 1000;



static void setup_timer();
static void *timer_handler(void *ptr);

pthread_t timer_thread;
pthread_cond_t timer_signal_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t timer_signal_mutex = PTHREAD_MUTEX_INITIALIZER;


static char timer_disable = 0;

static px_cameraid cameraid = PX_FRONT_CAM;

double get_time() {
    struct timeval  tv;
    gettimeofday(&tv, NULL);

    return (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ; 
}


static void setup_timer() {
    timer_thread = pthread_create(&timer_thread, NULL, timer_handler, NULL);
    pthread_detach(timer_thread);
}

void *timer_handler(void *ptr) {
    if(timer_disable == 1) {
        return;
    }
    struct timespec _t;
    clock_gettime(CLOCK_REALTIME, &_t);
    for (;;) {
        pxset_keepalive();
        //pxset_systemlog();
        pxset_img_seq(cameraid);  

        px_selfstate st;
        pxget_selfstate(&st);
        
        static unsigned long msec_cnt = 0;
        msec_cnt++;
        if(!(msec_cnt % 3)){
            //printf("%.2f %.2f %.2f | %.2f %.2f %.2f | %.2f | %d\n",st.degx,st.degy,st.degz,st.vision_tx,st.vision_ty,st.vision_tz,st.height,0);
        } 

        static int prev_operatemode = PX_HALT;
        if((prev_operatemode == PX_UP) && (pxget_operate_mode() == PX_HOVER)) {
            pxset_visioncontrol_xy(st.vision_tx,st.vision_ty);
        }
        prev_operatemode = pxget_operate_mode();  

        if(pxget_whisle_detect() == 1) {
            if(pxget_operate_mode() == PX_HOVER) {
                pxset_operate_mode(PX_DOWN);
            }      
            else if(pxget_operate_mode() == 0) {
                pxset_rangecontrol_z(150);
                pxset_operate_mode(PX_UP);       
            }      
        }

        if(pxget_battery() == 1) {
            timer_disable = 1;
            system("shutdown -h now\n");   
            exit(1);
        }
        struct timespec remains;
        _t.tv_nsec += 10000000;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &_t, &remains);
        clock_gettime(CLOCK_REALTIME, &_t);
    }
}


void init_phenox() {
    pxinit_chain();
    set_parameter();   
    printf("CPU0:Start Initialization. Please do not move Phenox.\n");
    while(!pxget_cpu1ready());
    setup_timer();
    printf("CPU0:Finished Initialization.\n");
    

    pxset_led(0,1); 
}

/*** plugin interface functions ***/

/******************************************************************************
Description.: parse input parameters
Input Value.: param contains the command line string and a pointer to globals
Return Value: 0 if everything is ok
******************************************************************************/
int input_init(input_parameter *param, int plugin_no)
{
    int i;
    plugin_number = plugin_no;
    if(pthread_mutex_init(&controls_mutex, NULL) != 0) {
        IPRINT("could not initialize mutex variable\n");
        exit(EXIT_FAILURE);
    }

    param->argv[0] = INPUT_PLUGIN_NAME;

    /* show all parameters for DBG purposes */
    for(i = 0; i < param->argc; i++) {
        DBG("argv[%d]=%s\n", i, param->argv[i]);
    }

    reset_getopt();
    while(1) {
        int option_index = 0, c = 0;
        static struct option long_options[] = {
            {"h", no_argument, 0, 0
            },
            {"help", no_argument, 0, 0},
            {"c", required_argument, 0, 0},
            {"camera", required_argument, 0, 0},
            {"q", required_argument, 0, 0},
            {"quality", required_argument, 0, 0},
            {0, 0, 0, 0}
        };

        c = getopt_long_only(param->argc, param->argv, "", long_options, &option_index);

        /* no more options to parse */
        if(c == -1) break;

        /* unrecognized option */
        if(c == '?') {
            help();
            return 1;
        }

        switch(option_index) {
            /* h, help */
        case 0:
        case 1:
            DBG("case 0,1\n");
            help();
            return 1;
            break;

            /* c, camera - camera 0 - front, 1 - bottom*/
        case 2:
        case 3:
            DBG("case 2,3\n");
            cameraid = atoi(optarg);
            if (cameraid != 0 && cameraid != 1) {
                return 1;
            }
            break;

            /* q, quality - jpegquality */
        case 4:
        case 5:
            DBG("case 4,5\n");
            jpegquality = atoi(optarg);

            break;


        default:
            DBG("default case\n");
            help();
            return 1;
        }
    }

    pglobal = param->global;

    IPRINT("delay.............: %i\n", delay);
    //IPRINT("resolution........: %s\n", pics->resolution);

    return 0;
}

/******************************************************************************
Description.: stops the execution of the worker thread
Input Value.: -
Return Value: 0
******************************************************************************/
int input_stop(int id)
{
    DBG("will cancel input thread\n");
    pthread_cancel(worker);
    pthread_cancel(timer_thread);

    return 0;
}

/******************************************************************************
Description.: starts the worker thread and allocates memory
Input Value.: -
Return Value: 0
******************************************************************************/
int input_run(int id)
{
    init_phenox();
    pglobal->in[id].buf = malloc(128 * 1024);
    if(pglobal->in[id].buf == NULL) {
        fprintf(stderr, "could not allocate memory\n");
        exit(EXIT_FAILURE);
    }

    if(pthread_create(&worker, 0, worker_thread, NULL) != 0) {
        free(pglobal->in[id].buf);
        fprintf(stderr, "could not start worker thread\n");
        exit(EXIT_FAILURE);
    }
    pthread_detach(worker);

    return 0;
}

/******************************************************************************
Description.: print help message
Input Value.: -
Return Value: -
******************************************************************************/
void help(void)
{
    fprintf(stderr, " ---------------------------------------------------------------\n" \
    " Help for input plugin..: "INPUT_PLUGIN_NAME"\n" \
    " ---------------------------------------------------------------\n" \
    " The following parameters can be passed to this plugin:\n\n" \
    " [-d | --delay ]........: delay to pause between frames\n" \
    " [-r | --resolution]....: can be 960x720, 640x480, 320x240, 160x120\n"
    " ---------------------------------------------------------------\n");
}

/******************************************************************************
Description.: copy a picture from testpictures.h and signal this to all output
              plugins, afterwards switch to the next frame of the animation.
Input Value.: arg is not used
Return Value: NULL
******************************************************************************/
void *worker_thread(void *arg)
{
    int i = 0;

    pthread_cleanup_push(worker_cleanup, NULL);

    const int ftmax = 200;
    px_imgfeature *ft =(px_imgfeature *)calloc(ftmax,sizeof(px_imgfeature));
    int ftstate = 0;


    IplImage *testImage;    
    int count = 0;

    double start_time;
    start_time = get_time();
    while(!pglobal->stop) {

        /* copy JPG picture to global buffer */
        if(pxget_imgfullwcheck(cameraid,&testImage) == 1) {
            if(ftstate == 1) {
                int ftnum = pxget_imgfeature(ft,ftmax);
                if(ftnum >= 0) {
                    for(i = 0;i < ftnum;i++) {
                        cvCircle(testImage,cvPoint((int)ft[i].pcx,(int)ft[i].pcy),2,CV_RGB(255,255,0),1,8,0);
                        cvCircle(testImage,cvPoint((int)ft[i].cx,(int)ft[i].cy),2,CV_RGB(0,255,0),1,8,0);
                        cvLine(testImage,cvPoint((int)ft[i].pcx,(int)ft[i].pcy),cvPoint((int)ft[i].cx,(int)ft[i].cy),CV_RGB(0,0,255),1,8,0);
                    }
                    ftstate = 0;
                }
            }
            if(pxset_imgfeature_query(cameraid) == 1) {
                ftstate = 1;
            }
            unsigned long buffsize = 128*1024;
            tjhandle tj_compressor = tjInitCompress();

            double encoding_time = get_time();
            if (tjCompress2(tj_compressor, (unsigned char*)testImage->imageData, 320, 0, 240, TJPF_BGR,
                      &(pglobal->in[plugin_number].buf), &buffsize, TJSAMP_420, jpegquality,
                      TJFLAG_NOREALLOC|TJFLAG_FASTDCT) == -1) {
                printf("%s\n", tjGetErrorStr());
            }
            encoding_time = get_time() - encoding_time;
            if (count % 30 == 0) {
              printf("%f: Image encoded, index: %d size: %ld encoding_time: %f\n", (get_time() - start_time)/1000.0,count++,buffsize, encoding_time);
            }

            tjDestroy(tj_compressor);

            pthread_mutex_lock(&pglobal->in[plugin_number].db);
            pglobal->in[plugin_number].size = buffsize;
            count++;      
            pthread_cond_broadcast(&pglobal->in[plugin_number].db_update);
            pthread_mutex_unlock(&pglobal->in[plugin_number].db);
        }
        usleep(1000);
    }

    IPRINT("leaving input thread, calling cleanup function now\n");
    pthread_cleanup_pop(1);

    return NULL;
}

/******************************************************************************
Description.: this functions cleans up allocated ressources
Input Value.: arg is unused
Return Value: -
******************************************************************************/
void worker_cleanup(void *arg)
{
    static unsigned char first_run = 1;

    if(!first_run) {
        DBG("already cleaned up ressources\n");
        return;
    }

    first_run = 0;
    DBG("cleaning up ressources allocated by input thread\n");

    if(pglobal->in[plugin_number].buf != NULL) free(pglobal->in[plugin_number].buf);
}

