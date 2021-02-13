#ifndef PIXYTARGET_H_
#define PIXYTARGET_H_

#include "Common.h"
#include <frc/I2C.h>
#include <PixyCameraI2C.h>


class TalonXX;

#define LEFT_RATIO_MIN          (0.1)
#define LEFT_RATIO_MAX          (0.45)
#define RIGHT_RATIO_MIN         (0.6)
#define RIGHT_RATIO_MAX         (1.0)
#define MAX_COMPOSITE_TARGETS   (5)
#define PIXY_TARGET_ARRAYSIZE   (10)
#define TELE_TARGET_ROTATE_MULTIPIER    (-0.01)

enum SideType
{
    LEFT_BLOCK,
    RIGHT_BLOCK,
    NEITHER
};
struct TargetBlock
{
    uint16_t signature; //Identification number for your object - you could set it in the pixymon
	uint16_t x; 
	uint16_t y; 
	uint16_t width;
	uint16_t height;
    SideType whichSide;
};

struct CompositeTarget
{
    int leftIndex;
    int rightIndex;
    int centerX;
    int centerY;
    int distance;
};

class PixyTarget
{
    private:
        TalonXX *mainRobot;
        PixyCameraI2C *pixyCam;

        int numBlocks;
        int previousNumBlocks;
        int numTargets;
        bool seenSyncFail;
        TargetBlock targetBlocks[PIXY_TARGET_ARRAYSIZE];
        CompositeTarget targets[MAX_COMPOSITE_TARGETS];
        

    public:
        PixyTarget(TalonXX* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);
        
        int GetCompositeTargetX(int chosenTarget);
        int GetCompositeTargetY(int chosenTarget);
        int GetCompositeTargetDistance(int chosenTarget);
        int GetNumTargets();
        void PrintCompositeTarget(int chosenTarget);
        void ResetTargets(void);

        void AnalyzeTarget(void);
        
        

        

    
    private:
        SideType IdentifySide(TargetBlock block);
        

        
};
#endif 
