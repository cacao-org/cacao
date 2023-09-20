/**
 * @file    ao188_preprocessor.c
 * @brief   Convert ao188 APD data into curvature + SH data
 *
 * Templated upon the RTS19 code.
 *
 */

#include <math.h>
#include <sys/socket.h> // For APD emergency shudown.
#include <arpa/inet.h> // For APD emergency shudown.

#include "CommandLineInterface/CLIcore.h"

static long NUM_APD_HOWFS = 188;
static long NUM_APD_LOWFS = 16;

static float CURVATURE_REGZ = 1.0;
static float SHACK_CENTROID_REGZ = 1.0;
static float one_sided_curv_integrator_gain = 0.01; // TODO FPS

// Local variables pointers
static char *apd_mat_name;
static long  fpi_wfsinsname;

static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".wfsin",
        "Wavefront sensor input",
        "wfsim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &apd_mat_name,
        &fpi_wfsinsname
    }
};
#include <stdio.h>
#include <sys/select.h>
int is_ready(int fd)
{
    fd_set fdset;
    struct timeval timeout;
    int ret;
    FD_ZERO(&fdset);
    FD_SET(fd, &fdset);
    timeout.tv_sec = 0;
    timeout.tv_usec = 1;
    return select(fd + 1, &fdset, NULL, NULL, &timeout) == 1 ? 1 : 0;
}

// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {

    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "ao188preproc", "AO188 APD Preprocessor", CLICMD_FIELDS_DEFAULTS
};

// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}


static errno_t apd_safety_execute(int lowfs_howfs)
{
    int sockfd;
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("Oh no APD emergency shutdown crapped!!!; can't create socket\n");
        return RETURN_FAILURE;
    }

    struct sockaddr_in server_addr;
    // Expecting elsewhere to manage SSH tunnels from localhost:18816/8 -> OBCP:10.0.0.6:18818
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    server_addr.sin_port = lowfs_howfs ? htons(18816) : htons(18818);

    if(connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        printf("Unable to connect on during APD emergency! err = %d %s\n", errno,
               strerror(errno));
        return RETURN_FAILURE;
    }
    const char msg_howfs[] = "howfs lash close\r";
    const char msg_lowfs[] = "lowfs lash close\r";
    const char *msg = lowfs_howfs ? msg_howfs : msg_lowfs;

    if(send(sockfd, msg, strlen(msg), 0) < 0)
    {
        printf("Unable to send message during APD emergency! err = %d %s\n", errno,
               strerror(errno));
        return RETURN_FAILURE;
    }
    close(sockfd);

    return RETURN_SUCCESS;
}

/*
Compute the curvature from the 188x2 APD curve-signed buffer.
NGL, assuming size is gonna be 188...
*/
static errno_t two_sided_curvature_compute(
    float *curvature,
    int16_t *apd_twosided,
    long size)
{
    float fp, fm;
    for(int k = 0; k < size; ++k)
    {
        fp = (float)apd_twosided[k];
        fm = (float)apd_twosided[k + size];
        curvature[k] = (fp - fm) / (fp + fm + CURVATURE_REGZ);
    }
    return RETURN_SUCCESS;
}

/*
One sided curvature updated each and every time
curv_sign = 0 or 1
*/
static errno_t one_sided_curvature_compute(
    float *curvature,
    int16_t *apd_onesided,
    float *apd_reference, // TODO must be updated.
    long size,
    int curv_sign)
{
    if(curv_sign == 0)
    {
        for(int k = 0; k < size; ++k)
        {
            curvature[k] = (apd_onesided[k] - apd_reference[k]) / apd_reference[k];
        }
    }
    else
    {
        for(int k = 0; k < size; ++k)
        {
            curvature[k] = (- apd_onesided[k] - apd_reference[k]) / apd_reference[k];
        }
    }
    return RETURN_SUCCESS;
}

static errno_t apd_integrator_update(
    float *apd_integrator,
    int16_t *apd_onesided,
    float integ_gain,
    long size
)
{
    for(int k = 0; k < size; ++k)
    {
        apd_integrator[k] *= 1.0 -
                             integ_gain; // TODO confcheck that gain < 1.0, maybe enfore much lower.
        apd_integrator[k] += integ_gain * apd_onesided[k];
    }
    return RETURN_SUCCESS;
}

struct __attribute__((__packed__)) LOWFS_INFO_STRUCT
{
    float local_tip[4];
    float local_tilt[4];

    float total_tip;
    float total_tilt;

    float focus;
};

static errno_t compute_lowfs_info(struct LOWFS_INFO_STRUCT *lowfs_struct,
                                  uint16_t *lowfs_apd)
{

    int subap_total = 0;
    int total_total = 0;

    for(int s = 0; s < 4; ++s)
    {
        subap_total = lowfs_apd[4 * s] + lowfs_apd[4 * s + 1] + lowfs_apd[4 * s + 2] +
                      lowfs_apd[4 * s + 3] + SHACK_CENTROID_REGZ;

        total_total += subap_total;

        lowfs_struct->local_tip[s] = (float)
                                     (+ lowfs_apd[4 * s + 0] - lowfs_apd[4 * s + 1]
                                      + lowfs_apd[4 * s + 2] - lowfs_apd[4 * s + 3]) /
                                     subap_total;
        lowfs_struct->local_tilt[s] = (float)
                                      (+ lowfs_apd[4 * s + 0] + lowfs_apd[4 * s + 1]
                                       - lowfs_apd[4 * s + 2] - lowfs_apd[4 * s + 3]) /
                                      subap_total;

        lowfs_struct->total_tip += lowfs_struct->local_tip[s];
        lowfs_struct->total_tilt += lowfs_struct->local_tilt[s];
    }

    lowfs_struct->total_tip /= 4.0;
    lowfs_struct->total_tilt /= 4.0;

    // Finally, the defocus - shouldn't there be a division by 4 here?
    lowfs_struct->focus = + lowfs_struct->local_tip[0] + lowfs_struct->local_tilt[0]
                          - lowfs_struct->local_tip[1] + lowfs_struct->local_tilt[1]
                          + lowfs_struct->local_tip[2] - lowfs_struct->local_tilt[2]
                          - lowfs_struct->local_tip[3] - lowfs_struct->local_tilt[3];

    return EXIT_SUCCESS;
}

static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // Since it's a fps PARAM_IMG, it's expected to be already loaded.
    IMGID apd_mat_in = mkIMGID_from_name(apd_mat_name);
    resolveIMGID(&apd_mat_in, ERRMODE_ABORT);

    float apd_integrator[NUM_APD_HOWFS];
    memset(apd_integrator, 0, NUM_APD_HOWFS * sizeof(float));

    int kw_idx_CURVSGN = -1; // TODO also make a friggin function.
    for(int k = 0; k < apd_mat_in.NBkw; ++k)
    {
        if(strcmp("_CURVSGN", apd_mat_in.im->kw[k].name) == 0)
        {
            kw_idx_CURVSGN = k;
            break;
        }
    }
    if(kw_idx_CURVSGN == -1)
    {
        printf("Fatal: must have _CURVSGN keyword in apd SHM.\n");
        fflush(stdout);
        return EXIT_FAILURE;
    }


    // Create curvature outputs
    uint32_t size_curvature = NUM_APD_HOWFS;
    IMGID curv_1k_doublesided = stream_connect_create_2D("curv_1kdouble",
                                size_curvature, 1,
                                _DATATYPE_FLOAT);
    IMGID curv_2k_doublesided = stream_connect_create_2D("curv_2kdouble",
                                size_curvature, 1,
                                _DATATYPE_FLOAT);
    IMGID curv_2k_singlesided = stream_connect_create_2D("curv_2ksingle",
                                size_curvature, 1,
                                _DATATYPE_FLOAT);



    IMGID lowfs_info = stream_connect_create_2D("lowfs_data", 11, 1,
                       _DATATYPE_FLOAT);
    struct LOWFS_INFO_STRUCT *lowfs_info_ptr = (struct LOWFS_INFO_STRUCT *)
            lowfs_info.im->array.F;

    // TODO Does procinfo need to be marked that apd_mat_in is the triggersname? YES!

    float frame_max = 0;
    float frame_mean = 0;
    float long_term_mean = 0;
    float long_term_mean_gain = 0.002;

    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT
    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        int curv_sign = apd_mat_in.im->kw[kw_idx_CURVSGN].value.numl;

        int16_t *apd_ptr = apd_mat_in.im->array.SI16 + curv_sign * apd_mat_in.size[0];
        int16_t *apd_lowfs_ptr = apd_ptr + NUM_APD_HOWFS;

        // Computations for safety
        frame_max = 0.0;
        frame_mean = 0.0;
        for(int kk = 0; kk < NUM_APD_HOWFS + NUM_APD_LOWFS; ++kk)
        {
            float val = (float) apd_ptr[kk];
            frame_max = val > frame_max ? val : frame_max;
            frame_mean += val;
        }
        frame_mean /= (NUM_APD_HOWFS + NUM_APD_LOWFS);
        long_term_mean = (1 - long_term_mean_gain) * long_term_mean + long_term_mean_gain * frame_mean;
        if(frame_max > 8000 || frame_mean > 4000 || long_term_mean > 2000)
        {
            fflush(stdout);
            apd_safety_execute(0); // lowfs
            apd_safety_execute(1); // howfs
            processloopOK = 0; // This is gonna quit
            // But the FPDP framegrabber is still running, so we can still get APD statistic to cntmon.
            for(int i = 0; i < 50; ++i)
            {
                printf("APD safety executing!!\n");
            }
            continue;
        }


        // Begin with LOWFS computations
        lowfs_info.im->md->write = 1;
        compute_lowfs_info(lowfs_info_ptr, apd_lowfs_ptr);
        processinfo_update_output_stream(processinfo, lowfs_info.ID);


        // TODO these 3 curvature computations
        // TODO can be re-ordered to optimize latency for the mode being used.


        // HOWFS curvature computations
        // TODO Pass keywords through. Or don't?
        curv_2k_doublesided.im->md->write = 1;
        two_sided_curvature_compute(curv_2k_doublesided.im->array.F, apd_mat_in.im->array.SI16, NUM_APD_HOWFS);
        processinfo_update_output_stream(processinfo, curv_2k_doublesided.ID);

        // Post outputs
        if(curv_sign == 1)
        {
            curv_1k_doublesided.im->md->write = 1;
            memcpy(curv_1k_doublesided.im->array.F, curv_2k_doublesided.im->array.F,
                   NUM_APD_HOWFS);
            processinfo_update_output_stream(processinfo, curv_1k_doublesided.ID);
        }

        curv_2k_singlesided.im->md->write = 1;
        // Get the latest side of the APD 216x2 buffer. WARNING: Size may be 217 if the curvature tag is embedded!
        // apd_mat_in.size[0] = 216 or 217 =/= NUM_APD_HOWFS.

        apd_integrator_update(apd_integrator, apd_ptr, NUM_APD_HOWFS, one_sided_curv_integrator_gain);
        one_sided_curvature_compute(curv_2k_singlesided.im->array.F, apd_ptr, apd_integrator, NUM_APD_HOWFS, curv_sign);

        processinfo_update_output_stream(processinfo, curv_2k_singlesided.ID);

    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    // We should probably clean close the SHMs?
    // Why doesn't anything in milk ever call ImageStreamIO_closeIm?

    DEBUG_TRACE_FEXIT();

    return RETURN_SUCCESS;
}

INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_IOtools__AO188Preproc()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}


/*
cacao << EOF
cacaoio.ao188preproc ..procinfo 1
cacaoio.ao188preproc ..triggersname apd
cacaoio.ao188preproc ..triggermode 3
cacaoio.ao188preproc ..loopcntMax -1
readshmim apd
cacaoio.ao188preproc apd apd
EOF
*/
