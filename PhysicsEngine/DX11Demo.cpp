
#include "DX11Demo.h"
#include <windowsx.h>

static XMMATRIX Convert(Matrix4 m)
{
	XMMATRIX xm;
	xm._11 = m.data[0];
	xm._12 = m.data[4];
	xm._13 = m.data[8];
	xm._14 = 0;

	xm._21 = m.data[1];
	xm._22 = m.data[5];
	xm._23 = m.data[9];
	xm._24 = 0;

	xm._31 = m.data[2];
	xm._32 = m.data[6];
	xm._33 = m.data[10];
	xm._34 = 0;

	xm._41 = m.data[3];
	xm._42 = m.data[7];
	xm._43 = m.data[11];
	xm._44 = 1;

	return xm;
}

template<typename T>
static T Clamp(const T& x, const T& low, const T& high)
{
	return x < low ? low : (x > high ? high : x);
}

namespace
{
	DX11Demo* dx11Demo = 0;
}

LRESULT CALLBACK
MainWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	return dx11Demo->MsgProc(hwnd, msg, wParam, lParam);
};

DX11Demo::DX11Demo(HINSTANCE hInstance)
	:mhAppInst(hInstance),
	mMainWndCaption(L"Physics Engine"),
	md3dDriverType(D3D_DRIVER_TYPE_HARDWARE),
	mClientWidth(800),
	mClientHeight(600),
	mEnable4xMsaa(false),
	mhMainWnd(0),
	m4xMsaaQuality(0),
	md3dDevice(0),
	md3dImmediateContext(0),
	mSwapChain(0),
	mDepthStencilBuffer(0),
	mRenderTargetView(0),
	mDepthStencilView(0),
	mBoxVB(0), mBoxIB(0), mFX(0), mTech(0),
	mfxWorldViewProj(0), mInputLayout(0),
	mTheta(1.3f*XM_PI), mPhi(0.4f*XM_PI), mRadius(35.0f)
{
	memset(&mScreenViewport, 0, sizeof(D3D11_VIEWPORT));

	mLastMousePos.x = 0;
	mLastMousePos.y = 0;

	XMMATRIX I = XMMatrixIdentity();
	XMStoreFloat4x4(&mView, I);
	XMStoreFloat4x4(&mProj, I);

	__int64 countsPerSec;
	QueryPerformanceFrequency((LARGE_INTEGER*)&countsPerSec);
	mSecondsPerCount = 1.0 / (double)countsPerSec;
	mPrevTime = 0;

	dx11Demo = this;

	InitPhysics();
}

DX11Demo::~DX11Demo()
{
	ReleaseCOM(mBoxVB);
	ReleaseCOM(mBoxIB);
	ReleaseCOM(mFX);
	ReleaseCOM(mInputLayout);

	ReleaseCOM(mRenderTargetView);
	ReleaseCOM(mDepthStencilView);
	ReleaseCOM(mSwapChain);
	ReleaseCOM(mDepthStencilBuffer);

	if (md3dImmediateContext)
		md3dImmediateContext->ClearState();

	ReleaseCOM(md3dImmediateContext);
	ReleaseCOM(md3dDevice);
}

void DX11Demo::OnMouseDown(WPARAM btnState, int x, int y)
{
	mLastMousePos.x = x;
	mLastMousePos.y = y;

	SetCapture(mhMainWnd);
}

void DX11Demo::OnMouseUp(WPARAM btnState, int x, int y)
{
	ReleaseCapture();
}

void DX11Demo::OnMouseMove(WPARAM btnState, int x, int y)
{
	if ((btnState & MK_LBUTTON) != 0)
	{
		float dx = XMConvertToRadians(0.25f*static_cast<float>(x - mLastMousePos.x));
		float dy = XMConvertToRadians(0.25f*static_cast<float>(y - mLastMousePos.y));

		mTheta += dx;
		mPhi += dy;

		mPhi = Clamp(mPhi, 0.1f, XM_PI - 0.1f);

	}
	else if ((btnState & MK_RBUTTON) != 0)
	{
		float dx = 0.01f*static_cast<float>(x - mLastMousePos.x);
		float dy = 0.01f*static_cast<float>(y - mLastMousePos.y);

		mRadius += dx - dy;
		mRadius = Clamp(mRadius, 3.0f, 50.0f);
	}

	mLastMousePos.x = x;
	mLastMousePos.y = y;
}

bool DX11Demo::InitMainWindow()
{
	WNDCLASS wc;
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = MainWndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = mhAppInst;
	wc.hIcon = LoadIcon(0, IDI_APPLICATION);
	wc.hCursor = LoadCursor(0, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)GetStockObject(NULL_BRUSH);
	wc.lpszMenuName = 0;
	wc.lpszClassName = L"D3DWndClassName";

	if (!RegisterClass(&wc))
	{
		MessageBox(0, L"RegisterClass Failed.", 0, 0);
		return false;
	}

	RECT R = { 0, 0, mClientWidth, mClientHeight };
	AdjustWindowRect(&R, WS_OVERLAPPEDWINDOW, false);
	int width = R.right - R.left;
	int height = R.bottom - R.top;

	mhMainWnd = CreateWindow(L"D3DWndClassName", mMainWndCaption.c_str(), 
		WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, width, height, 0, 0, mhAppInst, 0);
	
	if (!mhMainWnd)
	{
		MessageBox(0, L"CreateWindow Failed.", 0, 0);
		return false;
	}	

	ShowWindow(mhMainWnd, SW_SHOW);
	UpdateWindow(mhMainWnd);
	
	return true;
}

bool DX11Demo::InitDirect3D()
{

	UINT createDeviceFlags = 0;
#if defined(DEBUG) || defined(_DEBUG)  
	createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

	D3D_FEATURE_LEVEL featureLevel;
	HRESULT hr = D3D11CreateDevice(
		0,                 
		md3dDriverType,
		0,                 
		createDeviceFlags,
		0, 0,              
		D3D11_SDK_VERSION,
		&md3dDevice,
		&featureLevel,
		&md3dImmediateContext);

	if (FAILED(hr))
	{
		MessageBox(0, L"D3D11CreateDevice Failed.", 0, 0);
		return false;
	}

	if (featureLevel != D3D_FEATURE_LEVEL_11_0)
	{
		MessageBox(0, L"Direct3D Feature Level 11 unsupported.", 0, 0);
		return false;
	}


	HR(md3dDevice->CheckMultisampleQualityLevels(
		DXGI_FORMAT_R8G8B8A8_UNORM, 4, &m4xMsaaQuality));
	
	assert(m4xMsaaQuality > 0);

	DXGI_SWAP_CHAIN_DESC sd;
	sd.BufferDesc.Width = mClientWidth;
	sd.BufferDesc.Height = mClientHeight;
	sd.BufferDesc.RefreshRate.Numerator = 60;
	sd.BufferDesc.RefreshRate.Denominator = 1;
	sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	sd.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;

	if (mEnable4xMsaa)
	{
		sd.SampleDesc.Count = 4;
		sd.SampleDesc.Quality = m4xMsaaQuality - 1;
	}
	else
	{
		sd.SampleDesc.Count = 1;
		sd.SampleDesc.Quality = 0;
	}

	sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.BufferCount = 1;
	sd.OutputWindow = mhMainWnd;
	sd.Windowed = true;
	sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
	sd.Flags = 0;

	IDXGIDevice* dxgiDevice = 0;
	HR(md3dDevice->QueryInterface(__uuidof(IDXGIDevice), (void**)&dxgiDevice));

	IDXGIAdapter* dxgiAdapter = 0;
	HR(dxgiDevice->GetParent(__uuidof(IDXGIAdapter), (void**)&dxgiAdapter));

	IDXGIFactory* dxgiFactory = 0;
	HR(dxgiAdapter->GetParent(__uuidof(IDXGIFactory), (void**)&dxgiFactory));

	HR(dxgiFactory->CreateSwapChain(md3dDevice, &sd, &mSwapChain));

	ReleaseCOM(dxgiDevice);
	ReleaseCOM(dxgiAdapter);
	ReleaseCOM(dxgiFactory);

	OnResize();

	return true;
}

void DX11Demo::BuildGeometryBuffers()
{
	Vertex vertices[] =
	{
		{ XMFLOAT3(-0.5f, -0.5, -0.5f), Colors::White    },
		{ XMFLOAT3(-0.5f, +0.5, -0.5f), Colors::Black    },
		{ XMFLOAT3(+0.5f, +0.5f, -0.5f), Colors::Red     },
		{ XMFLOAT3(+0.5f, -0.5f, -0.5f), Colors::Green   },
		{ XMFLOAT3(-0.5f, -0.5f, +0.5f), Colors::Blue    },
		{ XMFLOAT3(-0.5f, +0.5f, +0.5f), Colors::Yellow  },
		{ XMFLOAT3(+0.5f, +0.5f, +0.5f), Colors::Cyan    },
		{ XMFLOAT3(+0.5f, -0.5f, +0.5f), Colors::Magenta }
	};

	D3D11_BUFFER_DESC vbd;
	vbd.Usage = D3D11_USAGE_IMMUTABLE;
	vbd.ByteWidth = sizeof(Vertex) * 8;
	vbd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	vbd.CPUAccessFlags = 0;
	vbd.MiscFlags = 0;
	vbd.StructureByteStride = 0;
	D3D11_SUBRESOURCE_DATA vinitData;
	vinitData.pSysMem = vertices;
	HR(md3dDevice->CreateBuffer(&vbd, &vinitData, &mBoxVB));

	UINT indices[] = 
	{
		0, 1, 2,
		0, 2, 3,

		4, 6, 5,
		4, 7, 6,

		4, 5, 1,
		4, 1, 0,

		3, 2, 6,
		3, 6, 7,

		1, 5, 6,
		1, 6, 2,

		4, 0, 3,
		4, 3, 7
	};

	D3D11_BUFFER_DESC ibd;
	ibd.Usage = D3D11_USAGE_IMMUTABLE;
	ibd.ByteWidth = sizeof(UINT) * 36;
	ibd.BindFlags = D3D11_BIND_INDEX_BUFFER;
	ibd.CPUAccessFlags = 0;
	ibd.MiscFlags = 0;
	ibd.StructureByteStride = 0;
	D3D11_SUBRESOURCE_DATA iinitData;
	iinitData.pSysMem = indices;
	HR(md3dDevice->CreateBuffer(&ibd, &iinitData, &mBoxIB));
}

void DX11Demo::BuildFX()
{
	DWORD shaderFlags = 0;
#if defined( DEBUG ) || defined( _DEBUG )
	shaderFlags |= D3D10_SHADER_DEBUG;
	shaderFlags |= D3D10_SHADER_SKIP_OPTIMIZATION;
#endif

	ID3D10Blob* compiledShader = 0;
	ID3D10Blob* compilationMsgs = 0;
	HRESULT hr = D3DX11CompileFromFile(L"FX/Basic.fx", 0, 0, 0, "fx_5_0", shaderFlags,
		0, 0, &compiledShader, &compilationMsgs, 0);

	if (compilationMsgs != 0)
	{
		MessageBoxA(0, (char*)compilationMsgs->GetBufferPointer(), 0, 0);
		ReleaseCOM(compilationMsgs);
	}

	HR(D3DX11CreateEffectFromMemory(compiledShader->GetBufferPointer(), compiledShader->GetBufferSize(),
		0, md3dDevice, &mFX));

	ReleaseCOM(compiledShader);

	mTech = mFX->GetTechniqueByName("BasicTech");
	mfxWorldViewProj = mFX->GetVariableByName("gWorldViewProj")->AsMatrix();
}

void DX11Demo::BuildVertexLayout()
{
	D3D11_INPUT_ELEMENT_DESC vertexDesc[] =
	{
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
		{"COLOR",    0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0}
	};

	D3DX11_PASS_DESC passDesc;
	mTech->GetPassByIndex(0)->GetDesc(&passDesc);
	HR(md3dDevice->CreateInputLayout(vertexDesc, 2, passDesc.pIAInputSignature,
		passDesc.IAInputSignatureSize, &mInputLayout));
}

bool DX11Demo::Init()
{
	if (!InitMainWindow())
		return false;

	if (!InitDirect3D())
		return false;

	BuildGeometryBuffers();
	BuildFX();
	BuildVertexLayout();
	return true;
}

LRESULT DX11Demo::MsgProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
		case WM_SIZE:
			mClientWidth = LOWORD(lParam);
			mClientHeight = HIWORD(lParam);

			if (md3dDevice)
			{
				OnResize();
			}
			return 0;

		case WM_DESTROY:
			PostQuitMessage(0);
			return 0;

		case WM_GETMINMAXINFO:
			((MINMAXINFO*)lParam)->ptMinTrackSize.x = 400;
			((MINMAXINFO*)lParam)->ptMinTrackSize.y = 400;
			return 0;

		case WM_LBUTTONDOWN:
		case WM_MBUTTONDOWN:
		case WM_RBUTTONDOWN:
			OnMouseDown(wParam, GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
			return 0;
		case WM_LBUTTONUP:
		case WM_MBUTTONUP:
		case WM_RBUTTONUP:
			OnMouseUp(wParam, GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
			return 0;
		case WM_MOUSEMOVE:
			OnMouseMove(wParam, GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
			return 0;
	}

	return DefWindowProc(hwnd, msg, wParam, lParam);
}

void DX11Demo::OnResize()
{
	assert(md3dImmediateContext);
	assert(md3dDevice);
	assert(mSwapChain);

	ReleaseCOM(mRenderTargetView);
	ReleaseCOM(mDepthStencilView);
	ReleaseCOM(mDepthStencilBuffer);

	HR(mSwapChain->ResizeBuffers(1, mClientWidth, mClientHeight, DXGI_FORMAT_R8G8B8A8_UNORM, 0));
	ID3D11Texture2D* backBuffer;
	HR(mSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&backBuffer)));
	HR(md3dDevice->CreateRenderTargetView(backBuffer, 0, &mRenderTargetView));
	ReleaseCOM(backBuffer);

	D3D11_TEXTURE2D_DESC depthStencilDesc;

	depthStencilDesc.Width = mClientWidth;
	depthStencilDesc.Height = mClientHeight;
	depthStencilDesc.MipLevels = 1;
	depthStencilDesc.ArraySize = 1;
	depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;

	if (mEnable4xMsaa)
	{
		depthStencilDesc.SampleDesc.Count = 4;
		depthStencilDesc.SampleDesc.Quality = m4xMsaaQuality - 1;
	}
	else
	{
		depthStencilDesc.SampleDesc.Count = 1;
		depthStencilDesc.SampleDesc.Quality = 0;
	}

	depthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
	depthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	depthStencilDesc.CPUAccessFlags = 0;
	depthStencilDesc.MiscFlags = 0;

	HR(md3dDevice->CreateTexture2D(&depthStencilDesc, 0, &mDepthStencilBuffer));
	HR(md3dDevice->CreateDepthStencilView(mDepthStencilBuffer, 0, &mDepthStencilView));

	md3dImmediateContext->OMSetRenderTargets(1, &mRenderTargetView, mDepthStencilView);

	mScreenViewport.TopLeftX = 0;
	mScreenViewport.TopLeftY = 0;
	mScreenViewport.Width = static_cast<float>(mClientWidth);
	mScreenViewport.Height = static_cast<float>(mClientHeight);
	mScreenViewport.MinDepth = 0.0f;
	mScreenViewport.MaxDepth = 1.0f;

	md3dImmediateContext->RSSetViewports(1, &mScreenViewport);

	XMMATRIX P = XMMatrixPerspectiveFovLH(0.25f*XM_PI, static_cast<float>(mClientWidth) / mClientHeight, 1.0f, 1000.0f);
	XMStoreFloat4x4(&mProj, P);
}

void DX11Demo::UpdateScene(float dt)
{
	float x = mRadius * sinf(mPhi)*cosf(mTheta);
	float z = mRadius * sinf(mPhi)*sinf(mTheta);
	float y = mRadius * cosf(mPhi);

	XMVECTOR pos = XMVectorSet(x, y, z, 1.0f);
	XMVECTOR target = XMVectorZero();
	XMVECTOR up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

	XMMATRIX V = XMMatrixLookAtLH(pos, target, up);
	XMStoreFloat4x4(&mView, V);

	world.RunPhysics(dt);

	PhysicsTestCases();
}

void DX11Demo::DrawScene()
{
	md3dImmediateContext->ClearRenderTargetView(mRenderTargetView, reinterpret_cast<const float*>(&Colors::LightSteelBlue));
	md3dImmediateContext->ClearDepthStencilView(mDepthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	md3dImmediateContext->IASetInputLayout(mInputLayout);
	md3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	UINT stride = sizeof(Vertex);
	UINT offset = 0;
	md3dImmediateContext->IASetVertexBuffers(0, 1, &mBoxVB, &stride, &offset);
	md3dImmediateContext->IASetIndexBuffer(mBoxIB, DXGI_FORMAT_R32_UINT, 0);

	XMMATRIX view = XMLoadFloat4x4(&mView);
	XMMATRIX proj = XMLoadFloat4x4(&mProj);
	XMMATRIX viewProj = view * proj;

	D3DX11_TECHNIQUE_DESC techDesc;
	mTech->GetDesc(&techDesc);
	for (UINT p = 0; p < techDesc.Passes; ++p)
	{
		for (int i = 0; i < 2; i++)
		{
			XMMATRIX world = Convert(rigidBody[i]->GetTransform());
			XMMATRIX worldViewProj = world * viewProj;

			mfxWorldViewProj->SetMatrix(reinterpret_cast<float*>(&worldViewProj));
			mTech->GetPassByIndex(p)->Apply(0, md3dImmediateContext);
			md3dImmediateContext->DrawIndexed(36, 0, 0);
		}
	}

	HR(mSwapChain->Present(0, 0));
}

void DX11Demo::Run()
{
	MSG msg = { 0 };
	
	while (msg.message != WM_QUIT)
	{
		if (PeekMessage(&msg, 0, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			__int64 currTime;
			QueryPerformanceCounter((LARGE_INTEGER*)&currTime);
			
			mDeltaTime = (currTime - mPrevTime)*mSecondsPerCount;
			mPrevTime = currTime;

			UpdateScene(mDeltaTime);
			DrawScene();

		}
	}
}

void DX11Demo::InitPhysics()
{
	rigidBody[0] = new RigidBody();
	rigidBody[1] = new RigidBody();

	world.bodies.push_back(rigidBody[0]);
	world.bodies.push_back(rigidBody[1]);

	ResetRigidbodies();

	BoxCollider* b0 = new BoxCollider();
	BoxCollider* b1 = new BoxCollider();

	b0->halfSize = Vector3(0.5, 0.5, 0.5);
	b1->halfSize = Vector3(0.5, 0.5, 0.5);

	//SphereCollider* b0 = new SphereCollider();
	//SphereCollider* b1 = new SphereCollider();
	//b0->radius = 0.5;
	//b1->radius = 0.5;

	collider[0] = b0;
	collider[1] = b1;

	collider[0]->rigidBody = rigidBody[0];
	collider[1]->rigidBody = rigidBody[1];

	world.colliders.push_back(collider[0]);
	world.colliders.push_back(collider[1]);
}

void DX11Demo::PhysicsTestCases()
{
	//Collision behaviour also can be changed by changing globalFriction and globalRestitution in headers.h

	// Object's rotation and velocity will become zero due to damping
	if (GetAsyncKeyState('1') & 0x8000)
	{	
		ResetRigidbodies();

		rigidBody[0]->SetVelocity(5, 4, 0);
		rigidBody[0]->SetRotation(3, 4, 0);	
		rigidBody[0]->SetLinearDamping(0.5);
		rigidBody[0]->SetAngularDamping(0.5);
	}
	
	//Collision of two movable and rotatable objects 
	if (GetAsyncKeyState('2') & 0x8000)
	{
		ResetRigidbodies();

		rigidBody[0]->SetVelocity(4, 0, 0);
		rigidBody[0]->SetRotation(3, 4, 0);

		rigidBody[1]->SetAcceleration(-4, 0, 0);
		rigidBody[1]->SetRotation(0, -2, 4);
	}

	//Collision with immovable but rotatable object
	if (GetAsyncKeyState('3') & 0x8000)
	{
		ResetRigidbodies();

		rigidBody[0]->SetInverseMass(0);

		rigidBody[1]->SetVelocity(-4, 0, 0);
		rigidBody[1]->SetAcceleration(-2, 0, 0);
		rigidBody[1]->SetRotation(0, -2, 4);
	}

	//Collision with non rotatable but movable object
	if (GetAsyncKeyState('4') & 0x8000)
	{
		ResetRigidbodies();

		rigidBody[0]->SetInverseInertiaTensor(Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0));

		rigidBody[1]->SetVelocity(-4, 0, 0);
		rigidBody[1]->SetAcceleration(-2, 0, 0);
		rigidBody[1]->SetRotation(0, -2, 4);
	}

	//Collision with immovable and non rotatable object
	if (GetAsyncKeyState('5') & 0x8000)
	{
		ResetRigidbodies();

		rigidBody[0]->SetInverseMass(0);
		rigidBody[0]->SetInverseInertiaTensor(Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0));

		rigidBody[1]->SetVelocity(-4, 0, 0);
		rigidBody[1]->SetAcceleration(-2, 0, 0);
		rigidBody[1]->SetRotation(0, -2, 4);
	}

	//Collision of movable and non rotatable object & immovable and rotatable object
	if (GetAsyncKeyState('6') & 0x8000)
	{
		ResetRigidbodies();

		rigidBody[0]->SetVelocity(5,0,0);
		rigidBody[0]->SetInverseInertiaTensor(Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0));

		rigidBody[1]->SetInverseMass(0);
		rigidBody[1]->SetRotation(3, 4, 5);
	}

	//Custom setup
	if (GetAsyncKeyState('7') & 0x8000)
	{
		ResetRigidbodies();

		rigidBody[0]->SetPosition(-10, 0, 0);
		rigidBody[0]->SetVelocity(5, 0, 0);
		rigidBody[0]->SetAcceleration(0, 0, 0);
		rigidBody[0]->SetOrientation(1, 0, 0, 0);
		rigidBody[0]->SetRotation(0, 0, 0);
		rigidBody[0]->SetInverseMass(1);
		rigidBody[0]->SetInverseInertiaTensor(Matrix3());
		rigidBody[0]->SetLinearDamping(1);
		rigidBody[0]->SetAngularDamping(1);

		rigidBody[1]->SetPosition(0, 10, 0);
		rigidBody[1]->SetVelocity(0, -5, 0);
		rigidBody[1]->SetAcceleration(0, 0, 0);
		rigidBody[1]->SetOrientation(1, 0, 0, 0);
		rigidBody[1]->SetRotation(0, 0, 0);
		rigidBody[1]->SetInverseMass(1);
		rigidBody[1]->SetInverseInertiaTensor(Matrix3());
		rigidBody[1]->SetLinearDamping(1);
		rigidBody[1]->SetAngularDamping(1);
	}
	
	
}

void DX11Demo::ResetRigidbodies()
{
	for (int i = 0; i < 2; i++)
	{
		if (i)
			rigidBody[i]->SetPosition(10, 0, 0);
		else
			rigidBody[i]->SetPosition(-10, 0, 0);

		rigidBody[i]->SetVelocity(0, 0, 0);
		rigidBody[i]->SetAcceleration(0, 0, 0);

		rigidBody[i]->SetOrientation(1, 0, 0, 0);
		rigidBody[i]->SetRotation(0, 0, 0);

		rigidBody[i]->SetInverseMass(1);
		rigidBody[i]->SetInverseInertiaTensor(Matrix3());

		rigidBody[i]->SetLinearDamping(1);
		rigidBody[i]->SetAngularDamping(1);
	}
	
}

