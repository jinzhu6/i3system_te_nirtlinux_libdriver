/** @file
    @brief  This file defines abstract class for communication with Thermal Expert
*/
#pragma once

#ifndef i3system_TE_H
#define i3system_TE_H

#ifdef I3_LIB_EXPORT
#define I3_LIB_API __attribute__((visibility("default")))
#else
#define I3_LIB_API
#endif

#define MAX_USB_NUM 32
#define MAX_THREAD_NUM 8

namespace i3{
     /*! \brief struct for setting values of TE.
     *
     * This structure is used to save or get setting values of TE.
     */
    typedef struct TE_Setting{
        unsigned short colorMap;/**< Display colormap. */
        unsigned short brightness;/**< Brighness of image. */
        unsigned short contrast;/**< Contrast of image. */
        bool AGC;/**< Apply AGC or not. */
        bool AIE;/**< Apply AIE or not. */
        unsigned short AIE_Factor;/**< AIE factor to apply in AIE algorithm. */
        bool alarm;/**< Apply alarm or not. */
        unsigned short alarmTemp;/**< Limit temperature when alarm is on. */
        bool traceHot;/**< Trace hottest point or not. */
        bool traceCold;/**< Trace coldest point or not. */
        bool center;/**< Display center temperature or not. */
    }TE_SETTING;


    /*! \brief struct for getting connected TE info.
     *
     * This structure is used for ScanTE function. ScanTE function returns connected TE info in this structure.
     */
    typedef struct TEScanData{
        unsigned char bDevCon; /**< indicates if TE is connected or not. 0 means connected, otherwise not connected. */
        unsigned int nCoreID; /**< ID of connected TE */
        unsigned int nProdVer; /**<  Connected model. 1 for Q1, 2 for V1, 3 for EQ1, 4 for EV1*/
        TEScanData(){
            bDevCon = 0;
            nCoreID = 0;
            nProdVer = 0;
        }
    }SCANINFO;

    /*! \brief structure about TE connection state
     *
     * This struct is used in GetUsbState function. It stores data whether TE is removed or arrived.
     */
    typedef struct TEState{
        int nUsbState; /**< indicate if TE connection state is changed. 1 if TE arrived, 2 if TE removed  */
        int nUsbNum; /**< Device number of TE of which state is changed*/
    }
    TE_STATE;

    class ITE_A_Impl;

    /*! \brief Interface for communication with Thermal Expert Type A(EQ1, EV1)
     */
    class TE_A{
    public:
        /**
           @brief Constructor of TE_A class
        */
        TE_A(unsigned int _coreID, unsigned int _rcgData, int hnd_dev, int _modelNum);

        /**
           @brief Constructor of TE_A class
        */
        TE_A(unsigned int _coreID, unsigned int _rcgData, int hnd_dev, int _modelNum, float *_pfTempGain, float *_pfTempOffset, float _pfAmb_A[], float _pfAmb_B[], float *_pfCalibFpaTemp, void *_pSeeting);

        /**
           @brief Destructor of TE_A class
        */
        virtual ~TE_A();

        /**
           @brief Disconnect Thermal Expert
           @remark This function disconnect the Thermal Expert and delete data. Do this function after finishing the work.
        */
        virtual void CloseTE();


        /**
           @brief Receive image data
           @param pRecvImage : variable to get image data (384x288 for QVGA and 640x480 for VGA, 32bits)
           @param _AGC_On : if set true, AGC algorithm is applied. Otherwise, AGC is not applied.
           @return 1: Read image successfully
           @return 2: Read image of size 0
           @return 3: Recieved image size is not equal to requested size
           @return 4: Data read fail
           @remark This function receive data from Thermal Expert and calibrate to show image. It send image data to pRecvImage. AGC algorithm is applied depending on the value of _AGC_On.
        */
        virtual int RecvImage(unsigned short *pRecvImage, bool _AGC_On = false);

        //virtual int RecvImage(unsigned short *pRecvImage);

        //virtual int RecvImage(float *pRecvImage);

        virtual unsigned short CalcTemp(unsigned short _x, unsigned short _y);

        /**
           @brief Get temperature of 1 frame
           @param _pTempBuf : buffer to get temperature data (384x288 for QVGA and 640x480 for VGA)
           @remark This function returns temperature data for entire image. Returned values are modified to fit into 2byte. To get temperature, user should subtract 5000 and then divide by 100, i.e. (real temperature value) = (return value - 5000) / 100
        */
        virtual void CalcTemp(unsigned short *_pTempBuf);

        /**
           @brief Do calibration
           @return 1: success
           @return 2: fail
           @remark This function will recalculate offset data for calibration.
           @remark Before calibration, stop receiving image data from Thermal Expert. It closes shutter and recalculate offset to obtain uniform image.
        */
        virtual int ShutterCalibrationOn();


        /**
           @brief Get image width
           @return Image width
           @remark Return width of image, i.e. it returns 384 for QVGA and 640 for VGA
        */
        int GetImageWidth();

        /**
           @brief Get image height
           @return Image height
           @remark Return height of image, i.e. it returns 288 for QVGA and 480 for VGA
        */
        int GetImageHeight();

        /**
           @brief Reset TE engine
           @return true if reset signal is transferred successfully, otherwise return false
           @remark It sends signal to reset TE engine. After calling this function, TE will be rebooted.
        */
        bool ResetMainBoard();

        /**
           @brief Save current setting in TE
           @param _pSetting : Pointer of TE_SETTING struct which contains setting values to save
           @return true if save setting signal is transferred successfully, otherwise return false
           @remark It sends signal to save setting values in TE engine. Values in _pSetting struct will be saved. Saved setting is not applied until TE is reconnected.
        */
        bool SaveSetting(TE_SETTING *_pSetting);

        /**
           @brief Get current setting in TE
           @param _pSetting : Pointer of TE_SETTING struct to get setting values
           @remark It receive setting values which is saved in TE, and returns those values in _pSetting variable.
        */
        void GetSetting(TE_SETTING *_pSetting);

        /**
           @brief Update bad pixels
           @remark If some bad pixels are generated while operating, this function finds those pixels which have quite different value compared to nearby pixels, and correction algorithm will be applied afterwards. Also, it saves those pixels and save in dead.txt files.
        */
        void UpdateDead();

        /**
           @brief Set factor of standard deviation used in bad pixel update algorithm
           @param _in : Input factor
           @remark In bad pixel update algorithm, each pixel value is compared to nearby pixels. It is considered as bad pixel when the differece is greater than some reference. This function sets this reference factor. If higher value is set, then pixels which have greater difference with nearby pixels will be considered as bad pixels.
        */
        void SetStd(float _in);

        void SetEmissivity(float _emissivity);

        unsigned int GetID();

    private:
        /**
           @brief TE_A implementation class
        */
        ITE_A_Impl *m_pTE_Impl;
    };


    class ITE_B_Impl;

    /*! \brief Interface for communication with Thermal Expert Type B(Q1, EQ1, V1)
     */
    class TE_B {
    public:
        /**
           @brief Constructor of TE_B class
        */
        TE_B();

        /**
           @brief Constructor of TE_B class
        */
        TE_B(unsigned int _coreID, unsigned int _rcgData, int hnd_dev, int _modelNum);

        /**
           @brief Destructor of TE_B class
        */
        virtual ~TE_B();

        /**
           @brief Disconnect Thermal Expert
           @remark This function disconnect the Thermal Expert and delete data. Do this function after finishing the work.
        */
        virtual void	CloseTE();

        /**
           @brief Read flash data
           @return 1: Read successfully
           @return 2: Fail while reading dead
           @return 3: Fail while reading gain
           @return 4: Fail while reading offset
           @return 5p: Fail while reading multioffset
           @remark It needs to get calibration data before receiving image data.
           @remark Do this function to read flash data right after connecting TE.
        */
        virtual int	ReadFlashData();


        /**
           @brief Receive image data
           @param pRecvImage : variable to get image data (384x288 for QVGA and 640x480 for VGA, 32bits)
           @return 1: Read image successfully
           @return 2: Read image of size 0
           @return 3: Recieved image size is not equal to requested size
           @return 4: Data read fail
           @return 7: Flash Data is not read
           @remark This function receive data from Thermal Expert and calibrate to show image. It send image data to pRecvImage in which AGC algorithm is not applied. Gain and offset value should be applied in user side.
        */
        virtual int	RecvImage(float *pRecvImage);

        /**
           @brief Receive image data
           @param pRecvImage : variable to get image data (384x288 for QVGA and 640x480 for VGA, 16bits)
           @return 1: Read image successfully
           @return 2: Read image of size 0
           @return 3: Recieved image size is not equal to requested size
           @return 4: Data read fail
           @return 7: Flash Data is not read
           @remark This function receive data from Thermal Expert and calibrate to show image. ReadFlashData function should be called before this function is called. It applies AGC algorithm, and then send image data fitted to 16bits to input argument pRecvImage.
        */
        virtual int	RecvImage(unsigned short *pRecvImage);

        /**
           @brief Do calibration
           @return 1: success
           @return 2: fail
           @remark This function will recalculate offset data for calibration.
           @remark Before calibration, stop receiving image data from Thermal Expert, and set Thermal Expert to see uniform temperature scene. It will recalculate offset to obtain uniform image.
        */
        virtual int	ShutterCalibrationOn();

        /**
           @brief Calculate temperature
           @param _x : Horizontal position of target pixel (0~383)
           @param _y : Vertical position of target pixel (0~287)
           @param isAmbientCalibOn : determine whether temperature calibration will be done which consider ambient temperature (true: do calibration, false: do not calibrate). Set false for EQ1 for now.
           @return Temperature (Celsius)
           @return std::numeric_limits<float>::max() if _x or _y value is out of range
           @return -std::numeric_limits<float>::max() if flash data is not read before
           @remark Return temperature of target pixel (_x, _y) from recived image data.
        */
        virtual float	CalcTemp(int _x, int _y, bool isAmbientCalibOn = false);

        /**
           @brief Calculate temperature of all image pixels
           @param pRecvTemp : variable to get temperature data (384x288 for QVGA and 640x480 for VGA, 32bits)
           @remark Return temperature array of all pixels to pRecvTemp
        */
        virtual void	CalcEntireTemp(float *pRecvTemp, bool isAmbientCalibOn = false);


        /**
           @brief Update dead data
           @return 0 : fail to update dead
           @return 1 : succeed to update dead
           @remark In some cases, dead can be generated which does not exist before, so it is considered non-dead pixel in flash data. This function receive images and analyze if additional dead pixels exist. If dead pixels exist, it will update dead data.
        */
        virtual int	UpdateDead();

        /**
           @brief Get image width
           @return Image width
           @remark Return width of image, i.e. it returns 384 for QVGA and 640 for VGA
        */
        virtual int             GetImageWidth();

        /**
           @brief Get image height
           @return Image height
           @remark Return height of image, i.e. it returns 288 for QVGA and 480 for VGA
        */
        virtual int             GetImageHeight();

        void SetEmissivity(float _emissivity);

    private:
        /**
           @brief TE_B implementation class
        */

        ITE_B_Impl *m_pTE_Impl;
    };

/**
   @brief Check connected TE.
   @param pDevCon : Buffer to get scan results. It should MAX_USB_NUM size array.
   @return 1 : Scan successfully
   @return Otherwise : Failed to scan
   @remark This function scans connected TE and return the results. The result contains connected devices's ID, Product Model, and device number.
   @remark For each connected TE, a device number(0 ~ MAX_USB_NUM - 1) is assigned. TE which has device number i corresponeds to i-th element of pDevCon and its scaned result is stored in i-th element of pDevCon. This device number is used in other function as hnd_dev.
*/
    extern "C" I3_LIB_API int ScanTE(SCANINFO *pDevCon);


/**
   @brief Connect Thermal Expert of Type A model.
   @return  nullptr : fail to open TE.
   @return  TE_A*(not nullptr) : pointer of TE_A class
   @remark This function initialize libusb and make Thermal Expert ready to communicate with host. It must be called to communicate with TE.
*/
    //extern "C" I3_LIB_API TE_A* OpenTE_A(int _TENum, int hnd_dev = 0);
    extern "C" I3_LIB_API TE_A* OpenTE_A(int hnd_dev = 0);

    /**
       @brief Connect USB.
       @param hnd_dev : device number for TE to connect
       @return  nullptr : fail to open TE.
       @return  TE_B*(not nullptr) : pointer of TE_B class
       @remark This function initialize libusb and make Thermal Expert ready to communicate with host. It must be called to communicate with TE.
    */
    extern "C" I3_LIB_API TE_B* OpenTE_B(int hnd_dev = 0);

    /**
       @brief Check usb connection
       @return 0 : TE connection state is not changed
       @return 1 : When TE is arrived to usb port
       @return 2 : When TE is is removed
       @remark After calling OpenTE function, usb connection is being check in background. If TE is removed from usb port without calling CloseTE function, it will return 2. When TE is arrived in usb port it will return 1.
       @remark It will have last change state until user call this function. After calling, it will be set to 0.
    */
    extern "C" I3_LIB_API TE_STATE GetUsbState();
}


#endif

/*! \brief Example code for TE type A. */
/*! \page Example_A
 *  \include Example_A.cpp
 */
/*! \brief Example code for TE type B. */
/*! \page Example_B
 *  \include Example_B.cpp
 */
