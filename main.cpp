#include "stdafx.h"
#include "classes/system/Shader.h"
#include "classes/system/Scene.h"
#include "classes/system/FPSController.h"
#include "classes/buffers/StaticBuffer.h"
#include "classes/level/Leaf.h"
#include "classes/level/PathGenerator.h"
#include "classes/delaunay/delaunay.h"
#include "classes/level/TileMap.h"

bool Pause;
bool keys[1024] = {0};
int WindowWidth = 800, WindowHeight = 600;
bool EnableVsync = 1;
GLFWwindow* window;
stFPSController FPSController;

int TilesCount[2] = {30, 30};
glm::vec2 Edge(2, 2);
glm::vec2 TileSize(20, 20);
glm::vec2 MouseSceneCoord;

MShader Shader;
MScene Scene;

MStaticBuffer Buffer;
MStaticBuffer ClickBuffer;
MStaticBuffer PathBuffer;

MTileMap TileMap;

vector<NVector2> PathPoints;
MPathGenerator PathGenerator;

bool FillTileBuffer() {
	glm::vec3 Color;
	for(int i=0; i<TilesCount[0] - 1; i++) {
		for(int j=0; j<TilesCount[1] - 1; j++) {
			if(TileMap.GetValue(i, j) == 0) continue;
			if(TileMap.GetValue(i, j) == 0) continue;
			if(TileMap.GetValue(i, j) < 0) {
				//here must be switch with select of wall types
				Color = glm::vec3(0, 0, 1);
			}
			if(TileMap.GetValue(i, j) > 0) {
				//here must be switch with select of floor types
				Color = glm::vec3(1, 1, 1);
			}
			Buffer.AddVertex(glm::vec2(i * TileSize.x + Edge.x, j * TileSize.y + Edge.y), Color);
			Buffer.AddVertex(glm::vec2((i + 1) * TileSize.x - Edge.x, j * TileSize.y + Edge.y), Color);
			Buffer.AddVertex(glm::vec2((i + 1) * TileSize.x - Edge.x, (j + 1) * TileSize.y - Edge.y), Color);
			Buffer.AddVertex(glm::vec2(i * TileSize.x + Edge.x, (j + 1) * TileSize.y - Edge.y), Color);
		}
	}
	return true;
}

bool GenerateLevel() {
	Buffer.Clear();
	ClickBuffer.Clear();
	PathBuffer.Clear();
	PathGenerator.clearCollisions();
	TileMap.Clear();
	
	list<TNode<stLeaf>* > Tree;
	int MinLeafSize = 6;
	int MaxLeafSize = 20;
	int MinRoomSize = 3;
	
	//create tree
	if(MinRoomSize >= MinLeafSize || MinLeafSize >= MaxLeafSize) {
		cout<<"Wrong settings"<<endl;
		return 0;
	}
	if(!SplitTree(&Tree, TilesCount[0], TilesCount[1], MinLeafSize, MaxLeafSize)) return false;
	
	//create rooms and fill centers map
	glm::vec3 Color = glm::vec3(1, 1, 1);
	TNode<NRectangle2>* pRoomNode;
	map<glm::vec2, TNode<NRectangle2>*, stVec2Compare> NodesCenters;
	glm::vec2 Center;
	NRectangle2* pRoom;
	list<TNode<stLeaf>* >::iterator itl;
	int RoomsNumber = 0;
	for(itl = Tree.begin(); itl != Tree.end(); itl++) {
		pRoomNode = CreateRoomInLeaf(*itl, MinRoomSize);
		if(!pRoomNode) continue;
		pRoom = pRoomNode->GetValueP();
		if(!pRoom) continue;
		//add in map
		Center.x = (pRoom->Position.x + pRoom->Size.x * 0.5) * TileSize.x;
		Center.y = (pRoom->Position.y + pRoom->Size.y * 0.5) * TileSize.y;
		NodesCenters.insert(pair<glm::vec2, TNode<NRectangle2>* >(Center, pRoomNode));
		//add in buffer
		RoomsNumber ++;
		TileMap.SetRectangle(*pRoom, 1);
	}
	if(RoomsNumber < 2) {
		cout<<"Too few rooms: "<<RoomsNumber<<endl;
		return false;
	}
	
	//copy centers for triangulation
	map<glm::vec2, TNode<NRectangle2>*, stVec2Compare>::iterator itm;
	vector<glm::vec2> CentersPoints;
	for(itm = NodesCenters.begin(); itm != NodesCenters.end(); itm++) {
		CentersPoints.push_back(itm->first);
	}
	
	//triangulate by delaunay and geet mst
	MDelaunay Triangulation;
	vector<MTriangle> Triangles = Triangulation.Triangulate(CentersPoints);
	vector<MEdge> Edges = Triangulation.GetEdges();
	vector<MEdge> MST = Triangulation.CreateMSTEdges();
	
	//create halls
	vector<NRectangle2> Halls;
	TNode<NRectangle2>* pNode0;
	TNode<NRectangle2>* pNode1;
	for(int i=0; i<MST.size(); i++) {
		pNode0 = NodesCenters[MST[i].p1];
		pNode1 = NodesCenters[MST[i].p2];
		//Halls = CreateHalls1(pNode0->GetValueP(), pNode1->GetValueP());
		Halls = CreateHalls2(pNode0->GetValueP(), pNode1->GetValueP());
		for(int k=0; k<Halls.size(); k++) {
			TileMap.SetRectangle(Halls[k], 1);
		}
	}
	Halls.clear();
	
	MST.clear();
	Triangulation.Clear();
	Triangles.clear();
	Edges.clear();
	CentersPoints.clear();

	NodesCenters.clear();
	
	ClearTree(&Tree);
	
	//set collisions to path finder
	for(int i=0; i < TilesCount[0]; i++) {
		for(int j=0; j < TilesCount[1]; j++) {
			if(TileMap.GetValue(i,j) <= 0) PathGenerator.addCollision(NVector2(i, j));
		}
	}
	//create walls
	//later create floor
	TileMap.CreateWalls();
	//fill buffer based on 
	FillTileBuffer();
	
	if(!Buffer.Dispose()) return false;
	return true;
}

static void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
}

static void mousepos_callback(GLFWwindow* window, double x, double y) {
	MouseSceneCoord = Scene.WindowPosToWorldPos(x, y);
}

static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		int x = (int)(MouseSceneCoord.x / TileSize.x);
		int y = (int)(MouseSceneCoord.y / TileSize.y);
		
		glm::vec3 Color;
		
		if(ClickBuffer.GetBufferSize() / 4 == 2) {
			PathPoints.clear();
			ClickBuffer.Clear();
			PathBuffer.Clear();
		}
		
		PathPoints.push_back(NVector2(x, y));
		
		if(ClickBuffer.GetBufferSize() / 4 == 1) {			
			CoordinateList Path = PathGenerator.findPath(PathPoints[0], PathPoints[1]);
			if(NVector2Compare(PathPoints.back(), Path.back())) {
				Color = glm::vec3(0, 1, 0);
			    for(CoordinateList::iterator it = Path.begin(); it != Path.end(); it++) {
			        PathBuffer.AddVertex(glm::vec2(it->x * TileSize.x + Edge.x, it->y * TileSize.y + Edge.y), Color);
					PathBuffer.AddVertex(glm::vec2((it->x + 1) * TileSize.x - Edge.x, it->y * TileSize.y + Edge.y), Color);
					PathBuffer.AddVertex(glm::vec2((it->x + 1) * TileSize.x - Edge.x, (it->y + 1) * TileSize.y - Edge.y), Color);
					PathBuffer.AddVertex(glm::vec2(it->x * TileSize.x + Edge.x, (it->y + 1) * TileSize.y - Edge.y), Color);
			    }
				PathBuffer.Dispose();
			}
		}
		
		Color = glm::vec3(1, 0, 0);
		ClickBuffer.AddVertex(glm::vec2(x * TileSize.x, y * TileSize.y), Color);
		ClickBuffer.AddVertex(glm::vec2((x + 1) * TileSize.x, y * TileSize.y), Color);
		ClickBuffer.AddVertex(glm::vec2((x + 1) * TileSize.x, (y + 1) * TileSize.y), Color);
		ClickBuffer.AddVertex(glm::vec2(x * TileSize.x, (y + 1) * TileSize.y), Color);
		ClickBuffer.Dispose();
	}
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GLFW_TRUE);
		return;
	}
	if(key == 'R' && action == GLFW_PRESS) {
		GenerateLevel();
	}
}

bool InitApp() {
	LogFile<<"Starting application"<<endl;    
    glfwSetErrorCallback(error_callback);
    
    if(!glfwInit()) return false;
    window = glfwCreateWindow(WindowWidth, WindowHeight, "TestApp", NULL, NULL);
    if(!window) {
        glfwTerminate();
        return false;
    }
    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, mousepos_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwMakeContextCurrent(window);
    if(glfwExtensionSupported("WGL_EXT_swap_control")) {
    	LogFile<<"Window: V-Sync supported. V-Sync: "<<EnableVsync<<endl;
		glfwSwapInterval(EnableVsync);//0 - disable, 1 - enable
	}
	else LogFile<<"Window: V-Sync not supported"<<endl;
    LogFile<<"Window created: width: "<<WindowWidth<<" height: "<<WindowHeight<<endl;

	//glew
	GLenum Error = glewInit();
	if(GLEW_OK != Error) {
		LogFile<<"Window: GLEW Loader error: "<<glewGetErrorString(Error)<<endl;
		return false;
	}
	LogFile<<"GLEW initialized"<<endl;
	
	if(!CheckOpenglSupport()) return false;

	//shaders
	if(!Shader.CreateShaderProgram("shaders/main.vertexshader.glsl", "shaders/main.fragmentshader.glsl")) return false;
	if(!Shader.AddUnifrom("MVP", "MVP")) return false;
	LogFile<<"Shaders loaded"<<endl;

	//scene
	if(!Scene.Initialize(&WindowWidth, &WindowHeight)) return false;
	LogFile<<"Scene initialized"<<endl;

	//randomize
    srand(time(NULL));
    LogFile<<"Randomized"<<endl;
    
    //other initializations
    //init buffer
    if(!Buffer.Initialize()) return false;
    Buffer.SetPrimitiveType(GL_QUADS);
    if(!ClickBuffer.Initialize()) return false;
    ClickBuffer.SetPrimitiveType(GL_QUADS);
    if(!PathBuffer.Initialize()) return false;
    PathBuffer.SetPrimitiveType(GL_QUADS);
    //init map
    TileMap = MTileMap(TilesCount[0], TilesCount[1]);
	//path generator
	PathGenerator.setWorldSize(NVector2(TilesCount[0], TilesCount[1]));
	PathGenerator.setHeuristic(euclidean);
    PathGenerator.setDiagonalMovement(false);
    //generate level
	if(!GenerateLevel()) return false;
	
	//turn off pause
	Pause = false;
    
    return true;
}

void RenderStep() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(Shader.ProgramId);
	glUniformMatrix4fv(Shader.Uniforms["MVP"], 1, GL_FALSE, Scene.GetDynamicMVP());
	
	//draw functions
	Buffer.Begin();
		Buffer.Draw();
		PathBuffer.Draw();
		ClickBuffer.Draw();
	Buffer.End();
}

void ClearApp() {
	//clear funstions
	PathPoints.clear();
	PathGenerator.clearCollisions();
	Buffer.Close();
	ClickBuffer.Close();
	PathBuffer.Close();
	TileMap.Close();
	
	memset(keys, 0, 1024);
	Shader.Close();
	LogFile<<"Application: closed"<<endl;
}

int main(int argc, char** argv) {
	LogFile<<"Application: started"<<endl;
	if(!InitApp()) {
		ClearApp();
		glfwTerminate();
		LogFile.close();
		return 0;
	}
	FPSController.Initialize(glfwGetTime());
	while(!glfwWindowShouldClose(window)) {
		FPSController.FrameStep(glfwGetTime());
    	FPSController.FrameCheck();
		RenderStep();
        glfwSwapBuffers(window);
        glfwPollEvents();
	}
	ClearApp();
    glfwTerminate();
    LogFile.close();
}
