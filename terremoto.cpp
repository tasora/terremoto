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


ChSharedPtr<ChMaterialSurface> mmat;

	// Utility function. Create a tapered column as a faceted convex hull.
	// For convex hulls, you just need to build a vector of points, it does not matter the order,
	// because they will be considered 'wrapped' in a convex hull anyway.
 
ChSharedPtr<ChBody> create_column(
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
	

	//create a texture for the column
	ChSharedPtr<ChTexture> mtexturecolumns(new ChTexture());
	mtexturecolumns->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
	bodyColumn->AddAsset(mtexturecolumns);

	bodyColumn->SetMaterialSurface(mmat);

	mphysicalSystem.Add(bodyColumn);

	return bodyColumn;
}

ChSharedPtr<ChBody> create_brickcolumn(
	ChSystem& mphysicalSystem,
	ChCoordsys<> base_pos,
	int    col_nedges = 10,
	double col_radius_hi = 0.45,
	double col_radius_lo = 0.5,
	double col_height = 6,
	double col_density = 3000)
{

	double col_base = 0;

	std::vector< ChVector<> > mpoints;
	for (int i = 0; i< col_nedges; ++i)
	{
		double alpha = CH_C_2PI * ((double)i / (double)col_nedges); // polar coord
		double x = col_radius_hi * cos(alpha);
		double z = col_radius_hi * sin(alpha);
		double y = col_base + col_height;
		mpoints.push_back(ChVector<>(x, y, z));
	}
	for (int i = 0; i< col_nedges; ++i)
	{
		double alpha = CH_C_2PI * ((double)i / (double)col_nedges); // polar coord
		double x = col_radius_lo * cos(alpha);
		double z = col_radius_lo * sin(alpha);
		double y = col_base;
		mpoints.push_back(ChVector<>(x, y, z));
	}
	ChSharedPtr<ChBodyEasyConvexHull> bodyColumn(new ChBodyEasyConvexHull(
		mpoints,
		col_density,
		true,
		true));
	ChCoordsys<> cog_column(ChVector<>(0, col_base + col_height / 2, 0));
	ChCoordsys<> abs_cog_column = cog_column >> base_pos;
	bodyColumn->SetCoord(abs_cog_column);
	mphysicalSystem.Add(bodyColumn);

	//create a texture for the brickcolumn
	ChSharedPtr<ChTexture> mtexturecolumns(new ChTexture());
	mtexturecolumns->SetTextureFilename(GetChronoDataFile("orange.png"));
	bodyColumn->AddAsset(mtexturecolumns);

	bodyColumn->SetMaterialSurface(mmat);

	return bodyColumn;

}
   
 
ChFunction* create_motion(std::string filename_pos, double t_offset = 0, double factor =1.0)
{
	ChStreamInAsciiFile mstream(GetChronoDataFile(filename_pos).c_str());
	
	ChFunction_Recorder* mrecorder = new ChFunction_Recorder;
	
	while(!mstream.End_of_stream())
	{
		double time = 0;
		double value = 0;
		try
		{
			mstream >> time;
			mstream >> value;

			GetLog() << "  t=" << time << "  p=" << value << "\n";

			mrecorder->AddPoint(time + t_offset, value * factor);
		}
		catch(ChException myerror)
		{
			GetLog() << "  End parsing file " << GetChronoDataFile(filename_pos).c_str() << " because: \n  " << myerror.what() << "\n";
			break;
		}
	}
	GetLog() << "Done parsing. \n";

	return mrecorder;
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
	application.AddTypicalCamera(core::vector3df(1,1,-5), core::vector3df(3,3,0));		//to change the position of camera
	application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));
 
	// Create a shared material surface used by columns etc.
	mmat = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	mmat->SetFriction(0.6);
	//mmat->SetSpinningFriction(0.01);
	//mmat->SetRollingFriction(0.01);
	mmat->SetCompliance(0.00000002);
	mmat->SetDampingF(1.5);

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

	ChSharedPtr<ChBodyEasyBox> tableBody(new ChBodyEasyBox( 17,1,15,  3000,	true, true));
	tableBody->SetPos( ChVector<>(4.05,-0.5,0) );

	mphysicalSystem.Add(tableBody);

	// optional, attach a texture for better visualization
	ChSharedPtr<ChTexture> mtextureconcrete(new ChTexture());
    mtextureconcrete->SetTextureFilename(GetChronoDataFile("grass.png"));
	tableBody->AddAsset(mtextureconcrete);


	// Create the constraint between ground and table. If no earthquake, it just
	// keeps the table in position.

	ChSharedPtr<ChLinkLockLock> linkEarthquake(new ChLinkLockLock);
	linkEarthquake->Initialize(tableBody, floorBody, ChCoordsys<>(ChVector<>(0,0,0)) );

	double time_offset = 5.0; // begin earthquake after 5 s to allow stabilization of blocks after creation.
	double ampl_factor = 2.0; // use lower or greater to scale the earthquake.
	bool   use_barrier = true; // if true, the Barrier data files are used, otherwise the No_Barrier datafiles are used

	// Define the horizontal motion, on x:
	//ChFunction_Sine* mmotion_x = new ChFunction_Sine(0,1.6,0.5); // phase freq ampl, carachteristics of input motion
	ChFunction* mmotion_x    = create_motion("Time history 10x0.50 Foam (d=6 m)/Barrier_Uv.txt", time_offset, ampl_factor);
	ChFunction* mmotion_x_NB = create_motion("Time history 10x0.50 Foam (d=6 m)/No_Barrier_Uv.txt", time_offset, ampl_factor);

	// Define the vertical motion, on y:
	ChFunction* mmotion_y    = create_motion("Time history 10x0.50 Foam (d=6 m)/Barrier_Uh.txt", time_offset, ampl_factor);
	ChFunction* mmotion_y_NB = create_motion("Time history 10x0.50 Foam (d=6 m)/No_Barrier_Uh.txt", time_offset, ampl_factor);

	if (use_barrier)
	{
		linkEarthquake->SetMotion_X(mmotion_x);
		linkEarthquake->SetMotion_Y(mmotion_y);
	}
	else
	{
		linkEarthquake->SetMotion_X(mmotion_x_NB);
		linkEarthquake->SetMotion_Y(mmotion_y_NB);
	}

	mphysicalSystem.Add(linkEarthquake);


	// Pointers to some objects that will be plotted, for future use.
	ChSharedPtr<ChBody> plot_brick_1;
	ChSharedPtr<ChBody> plot_table;

	plot_table = tableBody; // others will be hooked later.


	// Create the elements of the model



	if (false)		//if it's typed "true", the simple temple will be generated
	{

		double spacing = 2.2;
		double density = 3000;
		int nedges=10;

		//to create pedestals

		//create pedestal1

		ChSharedPtr<ChBodyEasyBox> pedestal1(new ChBodyEasyBox(
			0.7, 0.1, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_pedestal1(ChVector<>(0, 0, 0));
		pedestal1->SetCoord(cog_pedestal1);

		mphysicalSystem.Add(pedestal1);

		//create a texture for the pedestal1
		ChSharedPtr<ChTexture> mtexturepedestal1(new ChTexture());
		mtexturepedestal1->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		pedestal1->AddAsset(mtexturepedestal1);


		//create pedestal2

		ChSharedPtr<ChBodyEasyBox> pedestal2(new ChBodyEasyBox(
			0.7, 0.1, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_pedestal2(ChVector<>(spacing, 0, 0));
		pedestal2->SetCoord(cog_pedestal2);

		mphysicalSystem.Add(pedestal2);

		//create a texture for the pedestal2
		ChSharedPtr<ChTexture> mtexturepedestal2(new ChTexture());
		mtexturepedestal2->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		pedestal2->AddAsset(mtexturepedestal2);


		//create pedestal3

		ChSharedPtr<ChBodyEasyBox> pedestal3(new ChBodyEasyBox(
			0.7, 0.1, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_pedestal3(ChVector<>(spacing * 2, 0, 0));
		pedestal3->SetCoord(cog_pedestal3);

		mphysicalSystem.Add(pedestal3);

		//create a texture for the pedestal3
		ChSharedPtr<ChTexture> mtexturepedestal3(new ChTexture());
		mtexturepedestal3->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		pedestal3->AddAsset(mtexturepedestal2);



		//to create columns

		//create column1

		ChCoordsys<> base_position1l(ChVector<>(0 * spacing, 0.1, 0));	//coordinate of the first group of columns, bottom
		create_column(mphysicalSystem, base_position1l, nedges, 0.283, 0.30, 0.95, density);

		ChCoordsys<> base_position1m(ChVector<>(0 * spacing, 1.05, 0));
		create_brickcolumn(mphysicalSystem, base_position1m, nedges, 0.251, 0.283, 1.9, density);		//coordinate of the first group of columns, middle

		ChCoordsys<> base_position1h(ChVector<>(0 * spacing, 2.95, 0));
		plot_brick_1 = create_column(mphysicalSystem, base_position1h, nedges, 0.25, 0.251, 0.30, density);		//coordinate of the first group of columns, top
		// NOTE!!!!! the "plot_brick_1" points to this created column chunk, and plot_brick_1 is the one that will be plotted!!

		//create column2

		ChCoordsys<> base_position2l(ChVector<>(1 * spacing, 0.1, 0));	//coordinate of the second group of columns, bottom
		create_column(mphysicalSystem, base_position2l, nedges, 0.293, 0.30, 0.32, density);

		ChCoordsys<> base_position2m(ChVector<>(1 * spacing, 0.42, 0));
		create_brickcolumn(mphysicalSystem, base_position2m, nedges, 0.266, 0.293, 1.66, density);		//coordinate of the second group of columns, middle

		ChCoordsys<> base_position2h(ChVector<>(1 * spacing, 2.08, 0));
		create_column(mphysicalSystem, base_position2h, nedges, 0.25, 0.266, 1.17, density);		//coordinate of the second group of columns, top


		//create column3

		ChCoordsys<> base_position3l(ChVector<>(2 * spacing, 0.1, 0));	//coordinate of the third group of columns, bottom
		create_column(mphysicalSystem, base_position3l, nedges, 0.285, 0.30, 1.35, density);

		ChCoordsys<> base_position3m(ChVector<>(2 * spacing, 1.45, 0));
		create_brickcolumn(mphysicalSystem, base_position3m, nedges, 0.251, 0.285, 1.55, density);		//coordinate of the third group of columns, middle

		ChCoordsys<> base_position3h(ChVector<>(2 * spacing, 3, 0));
		create_column(mphysicalSystem, base_position3h, nedges, 0.25, 0.251, 0.25, density);		//coordinate of the third group of columns, top



		//to create capitals

		//create capital1

		ChSharedPtr<ChBodyEasyBox> capital1(new ChBodyEasyBox(
			0.7, 0.25, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capital1(ChVector<>(0, 3.375, 0));
		capital1->SetCoord(cog_capital1);

		mphysicalSystem.Add(capital1);

		//create a texture for the capital1
		ChSharedPtr<ChTexture> mtexturecapital1(new ChTexture());
		mtexturecapital1->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capital1->AddAsset(mtexturecapital1);


		//create capital2

		ChSharedPtr<ChBodyEasyBox> capital2(new ChBodyEasyBox(
			0.7, 0.25, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capital2(ChVector<>(spacing, 3.375, 0));
		capital2->SetCoord(cog_capital2);

		mphysicalSystem.Add(capital2);

		//create a texture for the capital2
		ChSharedPtr<ChTexture> mtexturecapital2(new ChTexture());
		mtexturecapital2->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capital2->AddAsset(mtexturecapital2);


		//create capital3

		ChSharedPtr<ChBodyEasyBox> capital3(new ChBodyEasyBox(
			0.7, 0.25, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capital3(ChVector<>(2 * spacing, 3.375, 0));
		capital3->SetCoord(cog_capital3);

		mphysicalSystem.Add(capital3);

		//create a texture for the capital3
		ChSharedPtr<ChTexture> mtexturecapital3(new ChTexture());
		mtexturecapital3->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capital3->AddAsset(mtexturecapital3);


		//to create top beam


		ChSharedPtr<ChBodyEasyBox> topBeam(new ChBodyEasyBox(
			5.8, 0.75, 0.6, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_topBeam(ChVector<>(2.25, 3.875, 0));
		topBeam->SetCoord(cog_topBeam);

		mphysicalSystem.Add(topBeam);

		//create a texture for the topBeam
		ChSharedPtr<ChTexture> mtexturetopBeam(new ChTexture());
		mtexturetopBeam->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		topBeam->AddAsset(mtexturetopBeam);


		/*for (int icol = 0; icol <3; ++icol)
		{


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
		}*/

	}


	//if it's typed "false", the complex temple will be generated

	else
	
	{
		double spacing = 2.7;
		double density = 3000;
		int nedges=10;

	//to create "big" columns

		//create column1

		ChCoordsys<> base_position1l(ChVector<>(0 * spacing, 0, 0));	//coordinate of the first group of columns, bottom
		create_column(mphysicalSystem, base_position1l, nedges, 0.284, 0.30, 0.97, density);

		ChCoordsys<> base_position1m(ChVector<>(0 * spacing, 0.97, 0));
		create_column(mphysicalSystem, base_position1m, nedges, 0.265, 0.284, 1.13, density);		//coordinate of the first group of columns, middle

		ChCoordsys<> base_position1h(ChVector<>(0 * spacing, 2.1, 0));
		create_column(mphysicalSystem, base_position1h, nedges, 0.25, 0.265, 1.15, density);		//coordinate of the first group of columns, top


		//create column2

		ChCoordsys<> base_position2l(ChVector<>(1 * spacing, 0, 0));	//coordinate of the second group of columns, bottom
		create_column(mphysicalSystem, base_position2l, nedges, 0.283, 0.30, 1.05, density);

		ChCoordsys<> base_position2m(ChVector<>(1 * spacing, 1.05, 0));
		create_column(mphysicalSystem, base_position2m, nedges, 0.267, 0.283, 0.95, density);		//coordinate of the second group of columns, middle

		ChCoordsys<> base_position2h(ChVector<>(1 * spacing, 2, 0));
		create_column(mphysicalSystem, base_position2h, nedges, 0.25, 0.267, 1.25, density);		//coordinate of the second group of columns, top


		//create column3

		ChCoordsys<> base_position3l(ChVector<>(2 * spacing, 0, 0));	//coordinate of the third group of columns, bottom
		create_column(mphysicalSystem, base_position3l, nedges, 0.284, 0.30, 0.95, density);

		ChCoordsys<> base_position3m(ChVector<>(2 * spacing, 0.95, 0));
		create_column(mphysicalSystem, base_position3m, nedges, 0.264, 0.284, 1.20, density);		//coordinate of the third group of columns, middle

		ChCoordsys<> base_position3h(ChVector<>(2 * spacing, 2.15, 0));
		create_column(mphysicalSystem, base_position3h, nedges, 0.25, 0.264, 1.1, density);		//coordinate of the third group of columns, top


		//create column4

		ChCoordsys<> base_position4l(ChVector<>(3 * spacing, 0, 0));	//coordinate of the fourth group of columns, bottom
		create_column(mphysicalSystem, base_position4l, nedges, 0.278, 0.30, 1.33, density);

		ChCoordsys<> base_position4m(ChVector<>(3 * spacing, 1.33, 0));
		create_column(mphysicalSystem, base_position4m, nedges, 0.264, 0.278, 0.85, density);		//coordinate of the fourth group of columns, middle

		ChCoordsys<> base_position4h(ChVector<>(3 * spacing, 2.18, 0));
		create_column(mphysicalSystem, base_position4h, nedges, 0.25, 0.264, 1.07, density);		//coordinate of the fourth group of columns, top


	//to create capitals

		//create capital1

		ChSharedPtr<ChBodyEasyBox> capital1(new ChBodyEasyBox(
			0.7, 0.25, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capital1(ChVector<>(0, 3.375, 0));
		capital1->SetCoord(cog_capital1);

		mphysicalSystem.Add(capital1);

		//create a texture for the capital1
		ChSharedPtr<ChTexture> mtexturecapital1(new ChTexture());
		mtexturecapital1->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capital1->AddAsset(mtexturecapital1);


		//create capital2

		ChSharedPtr<ChBodyEasyBox> capital2(new ChBodyEasyBox(
			0.7, 0.25, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capital2(ChVector<>(spacing, 3.375, 0));
		capital2->SetCoord(cog_capital2);

		mphysicalSystem.Add(capital2);

		//create a texture for the capital2
		ChSharedPtr<ChTexture> mtexturecapital2(new ChTexture());
		mtexturecapital2->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capital2->AddAsset(mtexturecapital2);


		//create capital3

		ChSharedPtr<ChBodyEasyBox> capital3(new ChBodyEasyBox(
			0.7, 0.25, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capital3(ChVector<>(2 * spacing, 3.375, 0));
		capital3->SetCoord(cog_capital3);

		mphysicalSystem.Add(capital3);

		//create a texture for the capital3
		ChSharedPtr<ChTexture> mtexturecapital3(new ChTexture());
		mtexturecapital3->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capital3->AddAsset(mtexturecapital3);


		//create capital4

		ChSharedPtr<ChBodyEasyBox> capital4(new ChBodyEasyBox(
			0.7, 0.25, 0.7, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capital4(ChVector<>(3 * spacing, 3.375, 0));
		capital4->SetCoord(cog_capital4);

		mphysicalSystem.Add(capital4);

		//create a texture for the capital4
		ChSharedPtr<ChTexture> mtexturecapital4(new ChTexture());
		mtexturecapital4->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capital4->AddAsset(mtexturecapital4);


	//to create topBeam


		//create topBeam1

				ChSharedPtr<ChBodyEasyBox> topBeam1(new ChBodyEasyBox(
			0.6+3*spacing, 0.75, 0.6, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_topBeam1(ChVector<>(3.748, 3.875, 0));
		topBeam1->SetCoord(cog_topBeam1);

		mphysicalSystem.Add(topBeam1);

		//create a texture for the topBeam1
		ChSharedPtr<ChTexture> mtexturetopBeam1(new ChTexture());
		mtexturetopBeam1->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		topBeam1->AddAsset(mtexturetopBeam1);


		//create topBeam2
		
		ChSharedPtr<ChBodyEasyBox> topBeam2(new ChBodyEasyBox(
			3 * spacing, 0.45, 0.8, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_topBeam2(ChVector<>(3.7, 4.475, 0.1));
		topBeam2->SetCoord(cog_topBeam2);

		mphysicalSystem.Add(topBeam2);

		//create a texture for the topBeam2
		ChSharedPtr<ChTexture> mtexturetopBeam2(new ChTexture());
		mtexturetopBeam2->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		topBeam2->AddAsset(mtexturetopBeam2);


	//to create pedestals

		//create pedestal1
		ChCoordsys<> base_position1p(ChVector<>(0 * spacing, 4.7, 0));		//coordinate of the first pedestal of the "little" columns
		create_column(mphysicalSystem, base_position1p, nedges, 0.175, 0.25, 0.20, density);


		//create pedestal2
		ChCoordsys<> base_position2p(ChVector<>(1 * spacing, 4.7, 0));		//coordinate of the second pedestal of the "little" columns
		create_column(mphysicalSystem, base_position2p, nedges, 0.175, 0.25, 0.20, density);

		//create pedestal3
		ChCoordsys<> base_position3p(ChVector<>(2 * spacing, 4.7, 0));		//coordinate of the third pedestal of the "little" columns
		create_column(mphysicalSystem, base_position3p, nedges, 0.175, 0.25, 0.20, density);


	//to create "little" columns

		//create little column1

		ChCoordsys<> base_position1ll(ChVector<>(0 * spacing, 4.9, 0));	//coordinate of the first group of "little" columns, bottom
		create_column(mphysicalSystem, base_position1ll, nedges, 0.164, 0.175, 1.14, density);

		ChCoordsys<> base_position1lm(ChVector<>(0 * spacing, 6.04, 0));
		plot_brick_1 = create_column(mphysicalSystem, base_position1lm, nedges, 0.15, 0.164, 1.41, density);		//coordinate of the first group of "little" columns, middle
		// NOTE!!!!! the "plot_brick_1" points to this created column chunk, and plot_brick_1 is the one that will be plotted!!
		

		//create little column2

		ChCoordsys<> base_position2ll(ChVector<>(1 * spacing, 4.9, 0));	//coordinate of the second group of "little" columns, bottom
		create_column(mphysicalSystem, base_position2ll, nedges, 0.171, 0.175, 0.48, density);

		ChCoordsys<> base_position2lm(ChVector<>(1 * spacing, 5.38, 0));
		create_column(mphysicalSystem, base_position2lm, nedges, 0.157, 0.171, 1.44, density);		//coordinate of the second group of "little" columns, middle

		ChCoordsys<> base_position2lh(ChVector<>(1 * spacing, 6.82, 0));
		create_column(mphysicalSystem, base_position2lh, nedges, 0.150, 0.157, 0.63, density);		//coordinate of the second group of "little" columns, top


		//create little column3

		ChCoordsys<> base_position3ll(ChVector<>(2 * spacing, 4.9, 0));	//coordinate of the third group of "little" columns, bottom
		create_column(mphysicalSystem, base_position3ll, nedges, 0.168, 0.175, 0.69, density);

		ChCoordsys<> base_position3lm(ChVector<>(2 * spacing, 5.59, 0));
		create_column(mphysicalSystem, base_position3lm, nedges, 0.15, 0.168, 1.86, density);		//coordinate of the third group of "little" columns, middle

		
		//to create capitals of little columns

		//create capital1

		ChSharedPtr<ChBodyEasyBox> capitall1(new ChBodyEasyBox(
			0.45, 0.25, 0.45, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capitall1(ChVector<>(0, 7.595, 0));
		capitall1->SetCoord(cog_capitall1);

		mphysicalSystem.Add(capitall1);

		//create a texture for the capital1
		ChSharedPtr<ChTexture> mtexturecapitall1(new ChTexture());
		mtexturecapitall1->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capitall1->AddAsset(mtexturecapitall1);


		//create capital2

		ChSharedPtr<ChBodyEasyBox> capitall2(new ChBodyEasyBox(
			0.45, 0.25, 0.45, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capitall2(ChVector<>(spacing, 7.575, 0));
		capitall2->SetCoord(cog_capitall2);

		mphysicalSystem.Add(capitall2);

		//create a texture for the capital2
		ChSharedPtr<ChTexture> mtexturecapitall2(new ChTexture());
		mtexturecapitall2->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capitall2->AddAsset(mtexturecapitall2);


		//create capital3

		ChSharedPtr<ChBodyEasyBox> capitall3(new ChBodyEasyBox(
			0.45, 0.25, 0.45, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_capitall3(ChVector<>(2 * spacing, 7.615, 0));
		capitall3->SetCoord(cog_capitall3);

		mphysicalSystem.Add(capitall3);

		//create a texture for the capital3
		ChSharedPtr<ChTexture> mtexturecapitall3(new ChTexture());
		mtexturecapitall3->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		capitall3->AddAsset(mtexturecapitall3);

		
		/*for (int icol = 0; icol <3; ++icol)
		{


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
		}*/




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

		while (application.GetDevice()->run())
		{
			application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

			application.DrawAll();

			application.DoStep();

			application.GetVideoDriver()->endScene();
		}
	}

	return 0;
}
  
