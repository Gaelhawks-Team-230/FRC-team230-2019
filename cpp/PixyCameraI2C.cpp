/*
 * PixyCameraI2C.cpp
 *
 *  Created on: Jan 24, 2017
 *      Author: Gaelhawks
 */

#include "Common.h"
#include "PixyCameraI2C.h"


PixyCameraI2C::PixyCameraI2C(int address)
{
	 i2cAddress = address;
     pixyi2c = new frc::I2C(frc::I2C::kMXP, i2cAddress);

     // declare the object data variables
     LocalReset();
}

void PixyCameraI2C::LocalReset()
{
     isBlockDetected = false;
     blockCount = 0;
 	 ignoredTarget = false;
 	 skipStart = false;
 	 syncFailed = false;
	 lastPixyWord = 0xffff;
 	 printCount = 0;
 	 wordCount = 0;

 	 /*leftIndex = 0;
 	 rightIndex = 0;
 	 topIndex = 0;
 	 bottomIndex = 0;

 	 isHorizontal = false;*/
}

// the remainder of this snippet should be placed in a loop where the data is also used.
// a while loop is suggested where the loop exits when the target is identified or a break button is
// depressed on the OI

void PixyCameraI2C::UpdateDash()
{
    //frc::SmartDashboard::PutBoolean("Target Detected", TargetDetected());
    frc::SmartDashboard::PutNumber("Number of blocks Detected", GetNumBlocks());
}


bool PixyCameraI2C::StartTracking() //checks whether if it is start of the normal frame, CC frame, or the data is out of sync
{
	 int targetLoopCount = 0;
	 bool searchStartWord = true;
	 lastPixyWord = 0xffff;

	 //printf("SEARCHING for START WORD...\n");
	 while(searchStartWord)
	 {
		 currPixyWord = GetWord(); //This it the function right underneath
		 //printf("%d Current Word %x\n", wordCount, currPixyWord);
		 //wordCount++;
		 if (currPixyWord == 0 && lastPixyWord == 0)
		 {
			 isBlockDetected = false;
			 syncFailed = true;
			 //printf("Pixy: sync FAILURE...\n");
			 return false;
		 }
		 else if (currPixyWord==PIXY_START_WORD && lastPixyWord==PIXY_START_WORD)
		 {
			 typeOfBlock = NORMAL_BLOCK;
			 isBlockDetected = true;
			 //syncFailed = false;
			 //printf("Pixy: sync GOOD...\n");
			 return true;
		 }
		 else if (currPixyWord==PIXY_START_WORD_CC && lastPixyWord==PIXY_START_WORD)
		 {
			 typeOfBlock = CC_BLOCK;
			 isBlockDetected = true;
			 //syncFailed = false;
			 //printf("Pixy: sync CC block...\n");
			 return true;
		 }
		 else if (currPixyWord==PIXY_START_WORDX) //when byte recieved was 0x55aa instead of otherway around, the code syncs the byte
		 {
			 //printf("Pixy: reorder");
			 GetByte(); // resync
		 }
		 else
		 {
			 targetLoopCount++;
			 if(targetLoopCount > 50)
			 {
				 printf("Pixy: sync TIMEOUT...\n");
				 isBlockDetected = false;
				 searchStartWord = false;
				 return false;
			 }
		 }

		 lastPixyWord = currPixyWord;
	 }
	 //printf("Pixy: StartTracking() return false\n");
	 return false;
}

uint16_t PixyCameraI2C::GetWord() //Getting two Bytes from Pixy (The full information)
{
	unsigned char buffer[2] = {0, 0};

	pixyi2c->ReadOnly(2, buffer);
	return (buffer[1] << 8) | buffer[0]; //shift buffer[1] by 8 bits and add( | is bitwise or) buffer[0] to it
}

uint8_t PixyCameraI2C::GetByte()//gets a byte
{
	unsigned char buffer[1] = {0};

	pixyi2c->ReadOnly(1, buffer);
	return buffer[0];
}

int PixyCameraI2C::ProcessBlocks()
{
	 blocks[0] = {0}; //resets the array - clears out data from previous reading
	 uint8_t blockIndex;
	 uint16_t pixyWord, checksum, sum;
	 Block *block;

	 blockCount = 0;
	 //printf("\nSTART of GETBLOCKS\n");

	 if (skipStart)
	 {
		 skipStart = false;
		 isBlockDetected = true;
	 }
	 else
	 {  //when computer has not seen 0xaa55 (starting frame)
		if (StartTracking() == false)
		{
			isBlockDetected = false;
			//printf("No start frame\n");
			return -1;
	    }
	 }
	 //lastPixyWord = 0xffff;


	 //for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE;)
	 while(true)//Read until we break out at the end of the block
	 {
		checksum = GetWord();
		//printf("%d Check Sum %x\n", wordCount, checksum);
		//wordCount++;
		if (checksum == PIXY_START_WORD) // we've reached the beginning of the next frame - checking for 0xaa55
		{
		   skipStart = true; //starts this function
		   typeOfBlock = NORMAL_BLOCK;
		   if (syncFailed)
		   {
			   //printf("sync failed, skip block\n");
			   syncFailed = false;
			   return -1;
		   }
		   //printf("skip, block count %d\n", blockCount);
		   return blockCount;
		}
		else if (checksum == PIXY_START_WORD_CC) //we've reached the beginning of the next frame - checking for 0xaa56
		{
			skipStart = true;
			typeOfBlock = CC_BLOCK;
		    if (syncFailed)
		    {
		    	//printf("sync failed, skip block\n");
			    syncFailed = false;
			    return -1;
		    }
			//printf("CC skip\n");
			return blockCount;
		}
		else if (checksum == 0)
		{
			//printf("Checksum=0?? Return blockCount %d\n", blockCount);
		    return blockCount;
		}

		//if (blockCount>blockArraySize)
		   //resize();

		//block = blocks + blockCount;
		//if(blockCount < MAX_BLOCKS)
		if(blockCount < PIXY_MAXIMUM_ARRAYSIZE)
		{
			block = &(blocks[blockCount]);
		}
		else
		{
			//block = NULL;
		    //printf("block is NULL\n");
		    printf("TOO many blocks %d\n", blockCount);
		    blockCount--;
			block = &(blocks[blockCount]);
		}

		//The loop indexes through the entire block of data received for that target and stores it in the block structure
		for (blockIndex = 0, sum = 0; blockIndex < sizeof(Block)/sizeof(uint16_t); blockIndex++)
		{
		   if (typeOfBlock == NORMAL_BLOCK && blockIndex >= 5) // skip --if not an CC block, no need to consider angle
		   {
			  block->angle = 0;
			  break;
		   }
		   pixyWord = GetWord();
		   sum += pixyWord; //sum = w + sum
		   //printf("%d Pixy Word %x	Block Count %d\n", wordCount, pixyWord, blockCount);
		   //wordCount++;
		   if (block != NULL)
		   {
			   *((uint16_t *)block + blockIndex) = pixyWord; //converts block to integer value
		   }
		}
		if (checksum == sum)
		{
		   blockCount++;
		   //printf("Next block is %d\n", blockCount);
		}
		else
		{
		   //printf("Pixy: checksum error\n");
		}

		pixyWord = GetWord(); //when this is start of the frame
		//printf("%d Pixy Word2 %x	Block Count %d\n", wordCount, pixyWord, blockCount);
		//wordCount++;
		if (pixyWord == PIXY_START_WORD)
		{
		   //lastPixyWord = pixyWord; // allow it to see the double startword for a new frame
		   typeOfBlock = NORMAL_BLOCK;
		}
		else if (pixyWord == PIXY_START_WORD_CC)
		   typeOfBlock = CC_BLOCK;
		else
		{
		   syncFailed = false;
		   //printf("successful return %d blocks\n", blockCount);
		   return blockCount;
		}
	 }
	 printf("End: No Targets\n");
	 return 0;
}



void PixyCameraI2C::PrintBlock(int index)
{
	if(blockCount == 0)
	{
		//printf("NO TARGET!!! :(\n");
		return;
	}
	if(index < 0)
	{
		//printf("Composite Target:  numBlock: %d  x: %d y: %d width: %d height: %d Gap: %d  orientation: %s \n", target.numBlocks, target.xPosition, target.yPosition, target.targetWidth, target.targetHeight, target.targetGap, target.isHorizontal?"Horizontal":"Vertical");
		//printf("Composite Target:   numBlock: %d         x: %d          width: %d  \n", target.numBlocks, target.xPosition,  target.targetWidth);
		//printf("%d Composite Target:   numBlock: %d      top: %d     center: %d   \n", printCount, target.numBlocks, target.leftX,  target.yPosition);
	}
	else if(index < GetNumBlocks() && index >= 0)
	{
		 if(isBlockDetected)
		{
			if (blocks[index].signature>PIXY_MAX_SIGNATURE) // color code! (CC)
			{
			  //printf("CC block! numblock: %d  sig: %X (%d decimal) x: %d y: %d width: %d height: %d angle %d\n", blockCount, blocks[index].signature, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height, blocks[index].angle);
			}
			else // regular block.  Note, angle is always zero, so no need to print
			{
			  //printf("Block Num: %d numBlock: %d sig: %d x: %d y: %d width: %d height: %d\n", index, blockCount, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height); //prints out data to console
			  //printf("Target:       x: %d          y: %d  \n", blocks[index].x, blocks[index].y);
			}
			//Serial.print(buf);
		}
		else
		{
			//printf("No target detected (index: %d)\n", index);
		}
	}
	else
	{
		//printf("Invalid Target\n");
	}
}

int PixyCameraI2C::GetNumBlocks()
{
	return blockCount;
}


bool PixyCameraI2C::TargetDetected()
{
	if(blockCount < 1)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void PixyCameraI2C::ResyncCamera()
{
	//printf("Resyncing...");
	double cameraStartTime = frc::GetTime();
	skipStart = false;
	delete(pixyi2c);
    pixyi2c = new frc::I2C(frc::I2C::kMXP, i2cAddress);
    LocalReset();
	double cameraEndTime = frc::GetTime();
	//printf("Resync Time: %f\n", cameraEndTime - cameraStartTime);
}

//Added in 2019 start
int PixyCameraI2C::getBlockCount()
{
	return blockCount;
}

Block* PixyCameraI2C::getBlocks()
{
	return blocks;
}
