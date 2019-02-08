// Define which camera to use for vision
//#define USB_CAMERA
//#define AXIS_CAMERA
#if defined(USB_CAMERA) || defined(AXIS_CAMERA)
#define CAMERA
#else
#undef CAMERA
#endif
//Xbox Controllers
#define         XBOX_ONE                0
#define         XBOX_TWO                1

// CAN BUS IDs
#define         CAN_PDP                 0
#define         CAN_TALON_LF            3
#define         CAN_TALON_RF            4
#define         CAN_TALON_LR            1
#define         CAN_TALON_RR            2
#define         CAN_TALON_LEFT_ROLLER   6
#define         CAN_TALON_RIGHT_ROLLER  8
#define         CAN_TALON_ELEV          5
#define         CAN_TALON_WRIST         7
#define         CAN_PCM                 11

// DIO Channels

#define         NAVX_DIO

#ifdef RIO_DIO // default
#define         DIO_LT_FRONT_1          0
#define         DIO_LT_FRONT_2          1
#define         DIO_LT_FRONT_3          2
#define         DIO_LT_FRONT_4          3
#define         DIO_LT_FRONT_5          4
#define         DIO_LT_FRONT_6          5
#define         DIO_LT_BACK_1           6
#define         DIO_LT_BACK_2           
#endif



#ifdef NAVX_DIO // navx
#define         DIO_LT_FRONT_1          10
#define         DIO_LT_FRONT_2          11
#define         DIO_LT_FRONT_3          12
#define         DIO_LT_FRONT_4          13
#define         DIO_LT_FRONT_5          18
#define         DIO_LT_FRONT_6          19
#define         DIO_LT_BACK_1           20
#define         DIO_LT_BACK_2           21
#endif

#define         DIO_US_RANGE_TRIGGER        8
#define         DIO_US_RANGE_PULSE          9
#define         DIO_US_RANGE_TRIGGER_LEFT   6
#define         DIO_US_RANGE_PULSE_LEFT     7