//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - ChEasyBody objects
//     - collisions and contacts 
//     - imposing a ground-relative motion to a body
//  
//	 CHRONO 
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
   
 
#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "assets/ChTexture.h"
#include "motion_functions/ChFunction_Sine.h"
#include "unit_IRRLICHT/ChIrrApp.h"
 


// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;
         
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui; 


	// Utility function. Create a tapered column as a faceted convex hull.
	// For convex hulls, you just need to build a vector of points, it does not matter the order,
	// because they will be considered 'wrapped' in a convex hull anyway.
  
void create_column(
		ChSystem& mphysicalSystem, 
		ChCoordsys<> base_pos, 
		int    col_nedges= 10,
		double col_radius_hi= 0.45,
		double col_radius_lo= 0.5,
		double col_height=6,
		double col_density= 3000)
{
	
	double col_base=0;

	std::vector< ChVector<> > mpoints;
	for (int i=0; i< col_nedges; ++i)
	{
		double alpha = CH_C_2PI * ((double)i/(double)col_nedges); // polar coord
		double x = col_radius_hi * cos(alpha);
		double z = col_radius_hi * sin(alpha);
		double y = col_base + col_height;
		mpoints.push_back( ChVector<> (x,y,z) );
	}
	for (int i=0; i< col_nedges; ++i)
	{
		double alpha = CH_C_2PI * ((double)i/(double)col_nedges); // polar coord
		double x = col_radius_lo * cos(alpha);
		double z = col_radius_lo * sin(alpha);
		double y = col_base;
		mpoints.push_back( ChVector<> (x,y,z) );
	}
	ChSharedPtr<ChBodyEasyConvexHull> bodyColumn(new ChBodyEasyConvexHull(
							mpoints, 
							col_density, 
							true, 
							true));
	ChCoordsys<> cog_column(ChVector<>(0, col_base+col_height/2, 0));
	ChCoordsys<> abs_cog_column = cog_column >> base_pos;
	bodyColumn->SetCoord( abs_cog_column );
	mphysicalSystem.Add(bodyColumn);

	//create a texture for the column
	ChSharedPtr<ChTexture> mtexturecolumns(new ChTexture());
	mtexturecolumns->SetTextureFilename(GetChronoDataFile("cubetexture_pinkwhite.png"));
	bodyColumn->AddAsset(mtexturecolumns);

	
}
   
 
int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Collisions between objects",core::dimension2d<u32>(800,600),false); //screen dimensions

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(3,7,-10));		//to change the position of camera
	application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 35, 512, video::SColorf(0.6,0.8,1));
 
	// Create all the rigid bodies.

	// Create a floor that is fixed (that is used also to represent the aboslute reference)

	ChSharedPtr<ChBodyEasyBox> floorBody(new ChBodyEasyBox( 20,2,20,  3000,	false, true));		//to create the floor, false -> doesn't represent a collide's surface
	floorBody->SetPos( ChVector<>(0,-2,0) );
	floorBody->SetBodyFixed(true);		//SetBodyFixed(true) -> it's fixed, it doesn't move respect to the Global Position System

	mphysicalSystem.Add(floorBody);

	// optional, attach a texture for better visualization
	ChSharedPtr<ChTexture> mtexture(new ChTexture());
    mtexture->SetTextureFilename(GetChronoDataFile("blu.png"));		//texture in /data
	floorBody->AddAsset(mtexture);		//add texture to the system



	// Create the table that is subject to earthquake

	ChSharedPtr<ChBodyEasyBox> tableBody(new ChBodyEasyBox( 15,1,15,  3000,	true, true));
	tableBody->SetPos( ChVector<>(0,-0.5,0) );

	mphysicalSystem.Add(tableBody);

	// optional, attach a texture for better visualization
	ChSharedPtr<ChTexture> mtextureconcrete(new ChTexture());
    mtextureconcrete->SetTextureFilename(GetChronoDataFile("grass.png"));
	tableBody->AddAsset(mtextureconcrete);


	// Create the constraint between ground and table. If no earthquake, it just
	// keeps the table in position.

	ChSharedPtr<ChLinkLockLock> linkEarthquake(new ChLinkLockLock);
	linkEarthquake->Initialize(tableBody, floorBody, ChCoordsys<>(ChVector<>(0,0,0)) );

	ChFunction_Sine* mmotion_x = new ChFunction_Sine(0,1.6,0.5); // phase freq ampl, carachteristics of input motion
	linkEarthquake->SetMotion_X(mmotion_x);

	mphysicalSystem.Add(linkEarthquake);


	// Create few column chuncks
	double spacing = 1.0;
	double density = 3000;
	for (int icol = 0; icol <3; ++icol)
	{ 
		ChCoordsys<> base_position1( ChVector<>(icol * spacing, 0, 0) );
		create_column(mphysicalSystem, base_position1, 100, 0.45, 0.50, 1.0, density);

		ChCoordsys<> base_position2( ChVector<>(icol * spacing, 1.0, 0) );
		create_column(mphysicalSystem, base_position2, 100, 0.40, 0.45, 1.0, density);

		ChCoordsys<> base_position3( ChVector<>(icol * spacing, 2.0, 0) );
		create_column(mphysicalSystem, base_position3, 100, 0.35, 0.40, 1.0, density);

		ChCoordsys<> base_position4(ChVector<>(icol * spacing, 0, 2));
		create_column(mphysicalSystem, base_position4, 100, 0.45, 0.50, 1.5, density);

		ChCoordsys<> base_position5(ChVector<>(icol * spacing, 1.5, 2));
		create_column(mphysicalSystem, base_position5, 100, 0.40, 0.45, 1.5, density);

		ChCoordsys<> base_position6(ChVector<>(icol * spacing, 3, 2));
		create_column(mphysicalSystem, base_position6, 100, 0.35, 0.40, 1.5, density);

		ChCoordsys<> base_position7(ChVector<>(icol * spacing, 0, 4));
		create_column(mphysicalSystem, base_position7, 100, 0.45, 0.50, 2.0, density);

		ChCoordsys<> base_position8(ChVector<>(icol * spacing, 2, 4));
		create_column(mphysicalSystem, base_position8, 100, 0.40, 0.45, 2.0, density);

		ChCoordsys<> base_position9(ChVector<>(icol * spacing, 4, 4));
		create_column(mphysicalSystem, base_position9, 100, 0.35, 0.40, 2.0, density);


		if (icol< 2)
		{
			ChSharedPtr<ChBodyEasyBox> bodyTop(new ChBodyEasyBox(
							spacing, 0.4, 0.6, // x y z sizes
							density, 
							true, 
							true));
				
			ChCoordsys<> cog_top(ChVector<>(icol * spacing + spacing/2, 3 + 0.4/2, 0));
			bodyTop->SetCoord( cog_top );

			mphysicalSystem.Add(bodyTop);

			//create a texture for the bodyTop
			ChSharedPtr<ChTexture> mtextureebodyTop(new ChTexture());
			mtextureebodyTop->SetTextureFilename(GetChronoDataFile("oldconcrete.jpg"));
			bodyTop->AddAsset(mtextureebodyTop);
		}

		if (icol< 2)
		{
			ChSharedPtr<ChBodyEasyBox> bodyTop(new ChBodyEasyBox(
				spacing, 1.4, 0.6, // x y z sizes
				density,
				true,
				true));

			ChCoordsys<> cog_top(ChVector<>(icol * spacing + spacing / 2, 6 + 1.4 / 2, 4));
			bodyTop->SetCoord(cog_top);

			mphysicalSystem.Add(bodyTop);

			//create a texture for the bodyTop
			ChSharedPtr<ChTexture> mtexturebodyTop(new ChTexture());
			mtexturebodyTop->SetTextureFilename(GetChronoDataFile("brick.jpg"));
			bodyTop->AddAsset(mtexturebodyTop);
		}
	}
 

	

	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes 
	application.AssetUpdateAll();

	// This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
	// for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)

	application.AddShadowAll();


	// Modify some setting of the physical system for the simulation, if you want
	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	mphysicalSystem.SetIterLCPmaxItersSpeed(50);
	mphysicalSystem.SetIterLCPmaxItersStab(5);

 
	//mphysicalSystem.SetUseSleeping(true);

	application.SetStepManage(true);
	application.SetTimestep(0.005);
	application.SetTryRealtime(true);

	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();
		
		application.DoStep();

		application.GetVideoDriver()->endScene();  
	}

	return 0;
}
  
