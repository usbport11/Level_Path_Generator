#include "TileMap.h"

MTileMap::MTileMap() {
	Map = NULL;
	Size.x = Size.y = 0;
}
MTileMap::MTileMap(int SizeX, int SizeY) {
	if(SizeX <= 0 || SizeY <= 0) return;
	Size.x = SizeX;
	Size.y = SizeY;
	Map = new char* [Size.x];
	for(int i=0; i<SizeY; i++) {
		Map[i] = new char[Size.y];
		memset(Map[i], 0, Size.y);
	}
}

MTileMap::MTileMap(NVector2 inSize) {
	if(inSize.x <= 0 || inSize.y <= 0) return;
	Size = inSize;
	Map = new char* [Size.x];
	for(int i=0; i<Size.y; i++) {
		Map[i] = new char[Size.y];
		memset(Map[i], 0, Size.y);
	}
}

void MTileMap::SetVector(int x, int y, char Value) {
	Map[x][y] = Value;
}

void MTileMap::SetVector(NVector2 Position, char Value) {
	Map[Position.x][Position.y] = Value;
}

void MTileMap::SetRectangle(int x, int y, int w, int h, char Value) {
	for(int i=x; i < x + w; i++) {
		for(int j=y; j < y + h; j++) {
			Map[i][j] = Value;
		}
	}
}

void MTileMap::SetRectangle(NRectangle2 Rectangle, char Value) {
	for(int i=Rectangle.Position.x; i < Rectangle.Position.x + Rectangle.Size.x; i++) {
		for(int j=Rectangle.Position.y; j < Rectangle.Position.y + Rectangle.Size.y; j++) {
			Map[i][j] = Value;
		}
	}
}

char MTileMap::GetValue(int x, int y) {
	return Map[x][y];
}

char MTileMap::GetValue(NVector2 Position) {
	return Map[Position.x][Position.y];
}

bool MTileMap::CreateWalls() {
	if(!Map) return false;
	if(Size.x <= 0 || Size.y <= 0) return false;
	for(int i=0; i<Size.x - 1; i++) {
		for(int j=0; j<Size.y - 1; j++) {
			//borders
			if(Map[i+1][j] > 0 && Map[i][j] <= 0) {
				Map[i][j] = WT_BOTTOM;
				Map[i+1][j] = FT_TOP;
			}
			if(Map[i+1][j] <=0 && Map[i][j] > 0) {
				Map[i+1][j] = WT_TOP;
				Map[i][j] = FT_BOTTOM;
			}
			if(Map[i][j+1] > 0 && Map[i][j] <= 0) {
				Map[i][j] = WT_LEFT;
				Map[i][j+1] = FT_RIGHT;
			}
			if(Map[i][j+1] <= 0 && Map[i][j] > 0) {
				Map[i][j+1] = WT_RIGHT;
				Map[i][j] = FT_LEFT;
			}
			
			//angles
			if(Map[i][j] <=0 && Map[i+1][j] <=0 && Map[i+1][j+1] <=0 && Map[i][j+1] > 0) {
				Map[i+1][j] = WT_LEFT_TOP;
				Map[i][j+1] = FT_RIGHT_BOTTOM;
			}
			if(Map[i+1][j] <= 0 && Map[i+1][j+1] <= 0 && Map[i][j+1] <= 0 && Map[i][j] > 0) {
				Map[i+1][j+1] = WT_RIGHT_TOP;
				Map[i][j] = FT_LEFT_BOTTOM;
			}
			if(Map[i][j] <= 0 && Map[i+1][j] <= 0 && Map[i][j+1] <= 0 && Map[i+1][j+1] > 0) {
				Map[i][j] = WT_LEFT_BOTTOM;
				Map[i+1][j+1] = FT_RIGHT_TOP;
			}
			if(Map[i][j] <= 0 && Map[i][j+1] <= 0 && Map[i+1][j+1] <= 0 && Map[i+1][j] > 0) {
				Map[i][j+1] = WT_RIGHT_BOTTOM;
				Map[i+1][j] = FT_LEFT_TOP;
			}
		}
	}
	return true;
}

bool MTileMap::CreateFloor() {
	return true;
}

void MTileMap::Clear() {
	for(int i=0; i<Size.x; i++) {
		memset(Map[i], 0, Size.y);
	}
}

void MTileMap::Close() {
	if(!Map) return;
	for(int i=0; i<Size.x; i++) {
		if(Map[i]) delete [] Map[i];
	}
	delete [] Map;
}
