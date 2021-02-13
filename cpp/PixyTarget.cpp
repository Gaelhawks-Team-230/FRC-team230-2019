#include "Common.h"
#include "PixyTarget.h"
#ifdef USE_PIXY_TARGET

PixyTarget::PixyTarget(TalonXX* pRobot)
{
    pixyCam = new PixyCameraI2C(TARGET_PIXY_I2C_ADDRESS);
    LocalReset();
}

void PixyTarget::LocalReset()
{
    numBlocks = 0;
    numTargets = 0;
    seenSyncFail = false;
}

void PixyTarget::StartingConfig()
{

}

void PixyTarget::StopAll()
{

}


SideType PixyTarget::IdentifySide(TargetBlock block)
{
    SideType blockSideType = NEITHER;
    if((((double)block.width/block.height) >= LEFT_RATIO_MIN) && (((double)block.width/block.height) <= LEFT_RATIO_MAX))
    {
        blockSideType = LEFT_BLOCK;
        //printf("Left   ");
    }
    if((((double)block.width/block.height) >= RIGHT_RATIO_MIN) && (((double)block.width/block.height) <= RIGHT_RATIO_MAX))
    {
        blockSideType = RIGHT_BLOCK;
        //printf("Right   ");
    }
    else
    {
        //printf("Neither   ");
    }
    //printf("width: %d   height: %d    width/height(ratio):  %f   X position:  %d  Y position:  %d\n",
    // block.width, block.height, (double)block.width/block.height, block.x, block.y);

    return blockSideType;
}

void PixyTarget::ResetTargets()
{
    numTargets = 0;
}
//Organizes block array brought in into order of smallest x to largest x
//Idenitifies  viable composite targets based on location of left and right blocks in relation
void PixyTarget::AnalyzeTarget()
{
    previousNumBlocks =  numBlocks;
    numBlocks = 0;
    numBlocks = pixyCam->ProcessBlocks();
    //numBlocks = pixyCam->getBlockCount();
    if (numBlocks == -1)  // sync failure
    {
       // just use blocks from previous frame
        seenSyncFail = true;
        numBlocks = previousNumBlocks;
        return;
    }
    if(seenSyncFail)
    {
        //if(numBlocks == 1)
        //{
            numBlocks = previousNumBlocks;
            seenSyncFail = false;
            return;
        //}
    }
     
    seenSyncFail = false;


    if (numBlocks > PIXY_TARGET_ARRAYSIZE)
    {
        numBlocks = PIXY_TARGET_ARRAYSIZE;  // prevent reading too many targets
    }
    numTargets = 0;
    Block *blockArray = pixyCam->getBlocks();
    //printf("Blocks coming in:\n");
    int arraySpot = 0;
    for(int i = 0; i < numBlocks; i++)
    {
        // figure out the spot in the array where this target belongs
        if(i == 0 || blockArray[i].x > blockArray[i-1].x)
        {          
            arraySpot = i;
        }
        else
        {
            for(int j = i-1; j >=0; j--)
            {
                if(blockArray[i].x < targetBlocks[j].x)
                {   
                    arraySpot = j;
                    for(int k = i; k >= j; k--)
                    {
                        if(k != i)
                        {   
                            targetBlocks[k+1] = targetBlocks[k];
                        }
                        else
                        {
                            targetBlocks[k] = targetBlocks[i];
                        }

                    }

                }
            }   
        }

        // put the target in its place
        targetBlocks[arraySpot].signature = blockArray[i].signature;
        targetBlocks[arraySpot].x = blockArray[i].x;
        targetBlocks[arraySpot].y = blockArray[i].y;
        targetBlocks[arraySpot].width = blockArray[i].width;
        targetBlocks[arraySpot].height = blockArray[i].height;

        targetBlocks[arraySpot].whichSide = IdentifySide(targetBlocks[arraySpot]);
    }


    // print targetblocks 
 /*   for(int m = 0; m < numBlocks; m++)
    {
        printf(" %d  %d  %d  %d  %f  \n",
        targetBlocks[m].x, targetBlocks[m].y,targetBlocks[m].width, targetBlocks[m].height, (double)targetBlocks[m].width/(double)targetBlocks[m].height);
    }
    printf("\n\n");*/


        // find the composite targets 
    for(int m = 0; m < numBlocks; m++)
    {
        //printf("Array Index:  %d   \n", m);
      /*  if (targetBlocks[m].whichSide == LEFT_BLOCK)
           {}//printf("LEFT  ");
        else if (targetBlocks[m].whichSide == RIGHT_BLOCK)
            {}//printf("RIGHT  ");}
        else {}
            //printf("NEITHER  ");
        //printf("width: %d   height: %d    (ratio):  %f   X position:  %d  Y position:  %d\n",
        //targetBlocks[m].width, targetBlocks[m].height, (double)targetBlocks[m].width/targetBlocks[m].height, targetBlocks[m].x, targetBlocks[m].y);
*/
        if((m != numBlocks -1) &&(targetBlocks[m].whichSide == LEFT_BLOCK && targetBlocks[m + 1].whichSide == RIGHT_BLOCK))
        {
            targets[numTargets].leftIndex = targetBlocks[m].x;
            targets[numTargets].rightIndex = targetBlocks[m +1].x;
            targets[numTargets].centerX = (targetBlocks[m+1].x + targetBlocks[m].x)/2;
            targets[numTargets].centerY = (targetBlocks[m+1].y + targetBlocks[m].y)/2;
            targets[numTargets].distance = targetBlocks[m+1].x - targetBlocks[m].x;
            //printf("Block 1:  %d   x location:  %d\n", m, targetBlocks[m].x);
            //printf("Block 2:  %d   x location:  %d\n", m+1, targetBlocks[m+1].x);
            numTargets++;

            //PrintCompositeTarget(numTargets);
        }
        //printf("\n");
    }
}

//chosen target is expected to be 1 for first target, 2 for second and so on from left to right
void PixyTarget::PrintCompositeTarget(int chosenTarget)
{
    int tIndex = chosenTarget -1;
    printf("Composite target %d= left index:  %d   right index:  %d   centerX:  %d   centerY:  %d   distance:  %d\n", 
    chosenTarget, targets[tIndex].leftIndex, targets[tIndex].rightIndex, targets[tIndex].centerX, targets[tIndex].centerY, targets[tIndex].distance);
}

//chosen target is expected to be 1 for first target, 2 for second and so on from left to right
//(applies to all getCompositeTarget_____s)
int PixyTarget::GetCompositeTargetX(int chosenTarget)
{
    int targetCenterX = targets[chosenTarget - 1].centerX; 
    return targetCenterX;
}

int PixyTarget::GetCompositeTargetY(int chosenTarget)
{
    int targetCenterY = targets[chosenTarget - 1].centerY; 
    return targetCenterY;
}

int PixyTarget::GetCompositeTargetDistance(int chosenTarget)
{
    int targetDist = targets[chosenTarget - 1].distance; 
    return targetDist;
}

int PixyTarget::GetNumTargets()
{
    return numTargets;
}


//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void PixyTarget::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Number of blocks seen: ", numBlocks);
    frc::SmartDashboard::PutNumber("Number of composite targets: ", numTargets);
    frc::SmartDashboard::PutNumber("Target Center; ", GetCompositeTargetX(1));
}

//Called every loop
void PixyTarget::Service()
{
    /*if(pMainRobot->IsPaused)
    {
        //Area where any motors woulds stop at their value 
    }*/
}
#endif