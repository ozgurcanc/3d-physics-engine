#include <iostream>
#include "DX11Demo.h"

//	Simulation of the some cases
//	
//	Press '1' -> Object's rotation and velocity will be zero due to damping
//	Press '2' -> Collision of two movable and rotatable objects 
//	Press '3' -> Collision with immovable but rotatable object
//	Press '4' -> Collision with non rotatable but movable object
//	Press '5' -> Collision with immovable and non rotatable object
//	Press '6' -> Collision of movable and non rotatable object & immovable and rotatable object
//	Press '7' -> Custom setup (Sliding)
//
//	Test cases can be found in PhysicsTestCases function in DX11Demo class
//	Collision behaviour also can be changed by changing globalFriction and globalRestitution in headers.h
//
//	Hold and move left mouse button   -> Rotating camera around the scene
//	Hold and move right mouse button  -> Zoom in-out
//
//
//	Additional Include Directories => Microsoft DirectX SDK (June 2010)\Include
//	Additional Library Directories => Microsoft DirectX SDK (June 2010)\Lib\x86
//	Additional Dependencies        => d3d11.lib;d3dx11.lib;D3DCompiler.lib;DxErr.lib;dxgi.lib;dxguid.lib;Effects11d.lib

int main()
{
	DX11Demo * dx11demo = new DX11Demo(GetModuleHandle(nullptr));

	if (dx11demo->Init()) dx11demo->Run();

	delete dx11demo;
}

