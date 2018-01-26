// V2DNav.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "DataType.h"
#include "MapGenerator.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <assert.h>


int main()
{
	std::ifstream fIn("E:\\Summary\\V2DNav\\C++\\V2DNav\\Debug\\MapData.pathfind",std::ios::in);
	V2DNav::PathBuildConfig BuildConfig;
	BuildConfig.Agent.AgentHeight = 132;
	BuildConfig.Agent.AgentWidth = 60;
	BuildConfig.Agent.StepHeight = 20;
	std::string Line;
	size_t FloorNum = 0;
	std::string MinX, MinY, MaxX, MaxY, bCanCross;

	std::vector<V2DNav::SFloor> Floors;
	while (std::getline(fIn, Line))
	{
		std::stringstream LineString(Line);
		LineString >> MinX;
		LineString >> MinY;
		LineString >> MaxX;
		LineString >> MaxY;
		LineString >> bCanCross;
		V2DNav::SFloor Floor;
		Floor.Min[0] = atoi(MinX.c_str()) + (BuildConfig.Agent.AgentWidth >> 1);
		Floor.Min[1] = atoi(MinY.c_str()) + (BuildConfig.Agent.AgentHeight >> 1);
		Floor.Max[0] = atoi(MaxX.c_str()) - (BuildConfig.Agent.AgentWidth >> 1);
		Floor.Max[1] = atoi(MaxY.c_str()) - (BuildConfig.Agent.AgentHeight >> 1);
		Floor.bCanCross = bool(atoi(bCanCross.c_str()));
		Floors.push_back(Floor);
		++FloorNum;
	}
	fIn.clear();
	fIn.close();

	BuildConfig.FloorNum = FloorNum;
	if (FloorNum)
	{
		BuildConfig.Floors = new V2DNav::SFloor[FloorNum];
		for (size_t i = 0; i < FloorNum; ++i)
		{
			BuildConfig.Floors[i] = Floors[i];
		}
	}

	const V2DNav::NavMap* pMap = V2DNav::BuildNavMap(BuildConfig);

    return 0;
}

