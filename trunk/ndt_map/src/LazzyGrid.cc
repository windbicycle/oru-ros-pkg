#include <LazzyGrid.hh>
#include <NDTCell.hh>
#include <climits>

using namespace std;
using namespace lslgeneric;

LazzyGrid::LazzyGrid(double cellSize):mp(new pcl::PointCloud<pcl::PointXYZ>()) {
    initialized = false;
    centerIsSet = false;
    sizeIsSet = false;
    cellSizeX = cellSizeY = cellSizeZ = cellSize;

}
	
LazzyGrid::LazzyGrid(LazzyGrid *prot) {

    sizeXmeters = prot->sizeXmeters;
    sizeYmeters = prot->sizeYmeters;
    sizeZmeters = prot->sizeZmeters;

    cellSizeX = prot->cellSizeX;
    cellSizeY = prot->cellSizeY;
    cellSizeZ = prot->cellSizeZ;

    sizeX = abs(ceil(sizeXmeters/cellSizeX));
    sizeY = abs(ceil(sizeYmeters/cellSizeY));
    sizeZ = abs(ceil(sizeZmeters/cellSizeZ));

    centerX = prot->centerX;
    centerY = prot->centerY;
    centerZ = prot->centerZ;

    protoType = prot->protoType->clone();
    initialize();

}

LazzyGrid::LazzyGrid(double _sizeXmeters, double _sizeYmeters, double _sizeZmeters,
		     double _cellSizeX, double _cellSizeY, double _cellSizeZ,
		     double _centerX, double _centerY, double _centerZ, 
		     Cell *cellPrototype ):mp(new pcl::PointCloud<pcl::PointXYZ>()) {

    sizeXmeters = _sizeXmeters;
    sizeYmeters = _sizeYmeters;
    sizeZmeters = _sizeZmeters;

    cellSizeX = _cellSizeX;
    cellSizeY = _cellSizeY;
    cellSizeZ = _cellSizeZ;

    sizeX = abs(ceil(sizeXmeters/cellSizeX));
    sizeY = abs(ceil(sizeYmeters/cellSizeY));
    sizeZ = abs(ceil(sizeZmeters/cellSizeZ));

    centerX = _centerX;
    centerY = _centerY;
    centerZ = _centerZ;

    protoType = cellPrototype->clone();
    initialize();


}

void LazzyGrid::setCenter(const double &cx, const double &cy, const double &cz) {
    centerX = cx;
    centerY = cy;
    centerZ = cz;
    
    centerIsSet = true;
    if(sizeIsSet) {
	initialize();
    }
}

void LazzyGrid::setSize(const double &sx, const double &sy, const double &sz) {
    
    sizeXmeters = sx;
    sizeYmeters = sy;
    sizeZmeters = sz;

    sizeX = abs(ceil(sizeXmeters/cellSizeX));
    sizeY = abs(ceil(sizeYmeters/cellSizeY));
    sizeZ = abs(ceil(sizeZmeters/cellSizeZ));
    
    sizeIsSet = true;
    if(centerIsSet) {
	initialize();
    }	
}

void LazzyGrid::initialize() {
    
    dataArray = new Cell***[sizeX];
    linkedCells = new bool**[sizeX];
    for(unsigned int i=0; i<sizeX; i++) {
	dataArray[i] = new Cell**[sizeY];
	linkedCells[i] = new bool*[sizeY];
	for(unsigned int j=0; j<sizeY; j++) {
	    dataArray[i][j] = new Cell*[sizeZ];
	    linkedCells[i][j] = new bool[sizeZ];
	    //set all cells to NULL
	    memset(dataArray[i][j],0,sizeZ*sizeof(Cell*));
	    memset(linkedCells[i][j],0,sizeZ*sizeof(bool));
	}
    }
    initialized = true;
}

LazzyGrid::~LazzyGrid() {
    if(initialized) {
	//go through all cells and delete the non-NULL ones
	for(unsigned int i=0; i<activeCells.size(); ++i) {
	    if(activeCells[i]) {
		delete activeCells[i];
	    }
	}
	for(unsigned int i=0; i<sizeX; i++) {
	    for(unsigned int j=0; j<sizeY; j++) {
		delete[] dataArray[i][j];
		delete[] linkedCells[i][j];
	    }
	    delete[] dataArray[i];
	    delete[] linkedCells[i];
	}
	delete[] dataArray;
	delete[] linkedCells;
	if(protoType!=NULL) {
	    delete protoType; 
	}
    }
}

Cell* LazzyGrid::getCellForPoint(const pcl::PointXYZ point) {
    
    int indX,indY,indZ;
    this->getIndexForPoint(point,indX,indY,indZ);

    if(indX >= sizeX || indY >= sizeY || indZ >= sizeZ) return NULL;
    if(!initialized) return NULL;
    if(dataArray == NULL) return NULL;
    if(dataArray[indX] == NULL) return NULL;
    if(dataArray[indX][indY] == NULL) return NULL;

//    cout<<"LZ: "<<indX<<" "<<indY<<" "<<indZ<<endl;
    return dataArray[indX][indY][indZ];
}

void LazzyGrid::addPoint(pcl::PointXYZ point) {

    int indX,indY,indZ;
    this->getIndexForPoint(point,indX,indY,indZ);
    pcl::PointXYZ centerCell;

    if(indX >= sizeX || indY >= sizeY || indZ >= sizeZ) {
	return;
    }
    if(!initialized) return;
    if(dataArray == NULL) return;
    if(dataArray[indX] == NULL) return;
    if(dataArray[indX][indY] == NULL) return;

    if(dataArray[indX][indY][indZ]==NULL) {
	//initialize cell
	dataArray[indX][indY][indZ] = protoType->clone();
	dataArray[indX][indY][indZ]->setDimensions(cellSizeX,cellSizeY,cellSizeZ);

	int idcX, idcY, idcZ;
	pcl::PointXYZ center;
	center.x = centerX;
	center.y = centerY;
	center.z = centerZ;
	this->getIndexForPoint(center, idcX,idcY,idcZ);	
	centerCell.x = centerX + (indX-idcX)*cellSizeX;
	centerCell.y = centerY + (indY-idcY)*cellSizeY;
	centerCell.z = centerZ + (indZ-idcZ)*cellSizeZ;
	dataArray[indX][indY][indZ]->setCenter(centerCell);
/*
	cout<<"center: "<<centerX<<" "<<centerY<<" "<<centerZ<<endl;
	cout<<"size  : "<<sizeX<<" "<<sizeY<<" "<<sizeZ<<endl;
	cout<<"p  : "<<point.x<<" "<<point.y<<" "<<point.z<<endl;
	cout<<"c  : "<<centerCell.x<<" "<<centerCell.y<<" "<<centerCell.z<<endl;
	cout<<"id : "<<indX<<" "<<indY<<" "<<indZ<<endl;
	cout<<"cs : "<<cellSizeX<<" "<<cellSizeY<<" "<<cellSizeZ<<endl;
*/
	activeCells.push_back(dataArray[indX][indY][indZ]);
    }
    dataArray[indX][indY][indZ]->addPoint(point);
}

std::vector<Cell*>::iterator LazzyGrid::begin() {
    //cout<<"active cells "<<activeCells.size()<<endl;
    return activeCells.begin();
}

std::vector<Cell*>::iterator LazzyGrid::end() {
    return activeCells.end();
}

int LazzyGrid::size() {
    return activeCells.size();
}

SpatialIndex* LazzyGrid::clone() {
    return new LazzyGrid(cellSizeX);
}

SpatialIndex* LazzyGrid::copy() {
    LazzyGrid *ret = new LazzyGrid(cellSizeX);
    std::vector<Cell*>::iterator it = this->begin();
    while(it!=this->end()) {
	NDTCell* r = dynamic_cast<NDTCell*> (*it);
	if(r == NULL) continue;
	for(int i=0; i<r->points.size(); i++) {
	    ret->addPoint(r->points[i]);
	}
	it++;
    }
    return ret;
}

void LazzyGrid::getNeighbors(pcl::PointXYZ point, const double &radius, std::vector<Cell*> &cells) {
    
    int indX,indY,indZ;
    this->getIndexForPoint(point, indX,indY,indZ);
    if(indX >= sizeX || indY >= sizeY || indZ >= sizeZ) {
	cells.clear();
	return;
    }

    for(int x = indX - radius/cellSizeX; x<indX+radius/cellSizeX; x++) {
	if(x < 0 || x >= sizeX) continue;
	for(int y = indY - radius/cellSizeY; y<indY+radius/cellSizeY; y++) {
	    if(y < 0 || y >= sizeY) continue;
	    for(int z = indZ - radius/cellSizeZ; z<indZ+radius/cellSizeZ; z++) {
		if(z < 0 || z >= sizeZ) continue;
		if(dataArray[x][y][z]==NULL) continue; 
		cells.push_back(dataArray[x][y][z]);
	    }
	}
    }

}

void LazzyGrid::getIndexForPoint(const pcl::PointXYZ& point, int &indX, int &indY, int &indZ) {
    indX = floor((point.x - centerX)/cellSizeX+0.5) + sizeX/2;
    indY = floor((point.y - centerY)/cellSizeY+0.5) + sizeY/2;
    indZ = floor((point.z - centerZ)/cellSizeZ+0.5) + sizeZ/2;
}
	
std::vector<NDTCell*> LazzyGrid::getClosestNDTCells(const pcl::PointXYZ point, double radius) {
    
    std::vector<int> id;
    std::vector<float> dist;
    int NCELLS = 4;
    id.reserve(NCELLS);
    dist.reserve(NCELLS);
    const pcl::PointXYZ pt(point);
    std::vector<NDTCell*> cells;
    if(!meansTree.nearestKSearch(pt,NCELLS,id,dist)) return cells;
    NDTCell *ret = NULL;
   
    for(int i=0; i<NCELLS; i++) { 
	//if(dist[i]>2*radius) continue;
	pcl::PointXYZ close = mp->points[id[i]];
	int indX,indY,indZ;
	this->getIndexForPoint(close, indX,indY,indZ);
	if(checkCellforNDT(indX,indY,indZ)) {
	    ret = dynamic_cast<NDTCell*> (dataArray[indX][indY][indZ]);
	    cells.push_back(ret);
	}	
    }
    return cells;
/*
    int indX,indY,indZ;
    this->getIndexForPoint(point, indX,indY,indZ);
    if(checkCellforNDT(indX,indY,indZ)) {
	ret = dynamic_cast<NDTCell*> (dataArray[indX][indY][indZ]);
	cells.push_back(ret);
    }	
    double maxNumberOfCells = 1; //radius/cellSizeX; //number of cells
    int indXn,indYn,indZn;

    //the strange thing for the indeces is for convenience of writing
    //basicly, we go through 2* the number of cells and use parity to 
    //decide if we subtract or add. should work nicely
    for(int x=1; x<2*maxNumberOfCells+2; x++) {
	indXn = (x%2 == 0) ? indX+x/2 : indX-x/2; 
	for(int y=1; y<2*maxNumberOfCells+2; y++) {
	    indYn = (y%2 == 0) ? indY+y/2 : indY-y/2; 
	    for(int z=1; z<2*maxNumberOfCells+2; z++) {
		indZn = (z%2 == 0) ? indZ+z/2 : indZ-z/2; 
		if(checkCellforNDT(indXn,indYn,indZn)) {
		    ret = dynamic_cast<NDTCell*> (dataArray[indXn][indYn][indZn]);
		    cells.push_back(ret);
		}
	    }
	}
    }

    return cells;
    */
}

void LazzyGrid::initKDTree() {
    
    NDTCell* ndcell = NULL;
    pcl::PointXYZ curr;
    Eigen::Vector3d m;
    pcl::PointCloud<pcl::PointXYZ> mc;
    
    for(int i=0; i<activeCells.size(); i++) {
	ndcell = dynamic_cast<NDTCell*> (activeCells[i]);
	if(ndcell == NULL) continue;
	if(!ndcell->hasGaussian) continue;
	m = ndcell->getMean();
	curr.x = m(0); 
	curr.y = m(1); 
	curr.z = m(2);
	mc.push_back(curr);	
    }

    if(mc.points.size() > 0) {
	*mp = mc;
	meansTree.setInputCloud(mp);
    }

}

NDTCell* LazzyGrid::getClosestNDTCell(pcl::PointXYZ point) {
    
    std::vector<int> id;
    std::vector<float> dist;
    id.reserve(1);
    dist.reserve(1);
    const pcl::PointXYZ pt(point);
    if(!meansTree.nearestKSearch(pt,1,id,dist)) return NULL;
    
    pcl::PointXYZ close = mp->points[id[0]];
    
    NDTCell *ret = NULL;
    int indX,indY,indZ;
    this->getIndexForPoint(close, indX,indY,indZ);
    if(checkCellforNDT(indX,indY,indZ)) {
	ret = dynamic_cast<NDTCell*> (dataArray[indX][indY][indZ]);
    }	
    return ret;

    /*
    std::vector<NDTCell*> cells;
    if(checkCellforNDT(indX,indY,indZ)) {
	ret = dynamic_cast<NDTCell*> (dataArray[indX][indY][indZ]);
	return ret;
    }	


    //tough luck. find the closest full cell
    double maxNumberOfCells = 3; //number of cells
    int indXn,indYn,indZn;

    bool doBreak = false;
    for(int i=1; i<= maxNumberOfCells; i++) {
	//the strange thing for the indeces is for convenience of writing
	//basicly, we go through 2* the number of cells and use parity to 
	//decide if we subtract or add. should work nicely
	for(int x=1; x<2*i+2; x++) {
	    indXn = (x%2 == 0) ? indX+x/2 : indX-x/2; 
	    for(int y=1; y<2*i+2; y++) {
		indYn = (y%2 == 0) ? indY+y/2 : indY-y/2; 
		for(int z=1; z<2*i+2; z++) {
		    indZn = (z%2 == 0) ? indZ+z/2 : indZ-z/2; 
		    if(checkCellforNDT(indXn,indYn,indZn)) {
			ret = dynamic_cast<NDTCell*> (dataArray[indXn][indYn][indZn]);
			cells.push_back(ret);
		    }
		}
	    }
	}

	double minDist = INT_MAX;
	Eigen::Vector3d tmean;
	pcl::PointXYZ pt = point;
*/	
    /*    //for caching
	      int idcX, idcY, idcZ;
	      pcl::PointXYZ center;
	      center.x = centerX;
	      center.y = centerY;
	      center.z = centerZ;
	      this->getIndexForPoint(center, idcX,idcY,idcZ);	
	      pt.x = centerX + (indX-idcX)*cellSizeX;
	      pt.y = centerY + (indY-idcY)*cellSizeY;
	      pt.z = centerZ + (indZ-idcZ)*cellSizeZ;

	 */   //
/*	for(int i=0; i<cells.size(); i++) {
	    tmean = cells[i]->getMean();
	    tmean(0) -= pt.x;
	    tmean(1) -= pt.y;
	    tmean(2) -= pt.z;
	    double d = tmean.norm();
	    if(d<minDist) {
		doBreak = true;
		minDist = d;
		ret = cells[i];
	    }
	}
	if(doBreak) {
	    break;
	} 
	
	cells.clear();

	//caching?
*/	/*  
	    if(indX < sizeX && indY < sizeY && indZ < sizeZ &&
	    indX >=0 && indY >=0 && indZ >=0 && ret!= NULL) {
	    dataArray[indX][indY][indZ] = ret;
	    linkedCells[indX][indY][indZ] = true;
	    }
	 */
/*    }
    return ret;
    */
}

bool LazzyGrid::checkCellforNDT(int indX, int indY, int indZ) {

    if(indX < sizeX && indY < sizeY && indZ < sizeZ &&
       indX >=0 && indY >=0 && indZ >=0) {
	if(dataArray[indX][indY][indZ]!=NULL) {
	    NDTCell* ret = dynamic_cast<NDTCell*> (dataArray[indX][indY][indZ]);
	    if(ret!=NULL) {
		if(ret->hasGaussian) {
		    return true;
		}
	    }
	}
    }
    return false;    
}

void LazzyGrid::setCellType(Cell *type) {
    if(type!=NULL) {
        protoType = type->clone();
    }
}
	
Cell* LazzyGrid::getCellAt(int indX, int indY, int indZ) {
    if(indX < sizeX && indY < sizeY && indZ < sizeZ &&
       indX >=0 && indY >=0 && indZ >=0) {
	return dataArray[indX][indY][indZ];
    }
    return NULL;
}
	
bool LazzyGrid::getLinkedAt(int indX, int indY, int indZ) {
    if(indX < sizeX && indY < sizeY && indZ < sizeZ &&
       indX >=0 && indY >=0 && indZ >=0) {
	return linkedCells[indX][indY][indZ];
    }
    return false;

}

void LazzyGrid::getCellSize(float &cx, float &cy, float &cz) {
    cx = cellSizeX;
    cy = cellSizeY;
    cz = cellSizeZ;
}

void LazzyGrid::getCenter(float &cx, float &cy, float &cz) {
    cx = centerX;
    cy = centerY;
    cz = centerZ;

}

void LazzyGrid::getGridSize(int &cx, int &cy, int &cz) {
    cx = sizeX;
    cy = sizeY;
    cz = sizeZ;
}
	
void LazzyGrid::writeLinksVRML(FILE* fout) {
  
    fprintf(fout,"Shape {\n\tgeometry IndexedLineSet {\n\tcoord Coordinate {\n\t point [\n\t");

    int n_lines = 0;
    pcl::PointXYZ centerCell; 
    pcl::PointXYZ center;
    for(int i=0; i<sizeX; i++) {
	for(int j=0; j<sizeY; j++) {
	    for(int k=0; k<sizeZ; k++) {
		if(getLinkedAt(i,j,k)) {
		    NDTCell* link = dynamic_cast<NDTCell*> (this->getCellAt(i,j,k));
		    if(link!=NULL) {
			int idcX, idcY, idcZ;
			center.x = centerX;
			center.y = centerY;
			center.z = centerZ;
			this->getIndexForPoint(center, idcX,idcY,idcZ);	
			centerCell.x = centerX + (i-idcX)*cellSizeX;
			centerCell.y = centerY + (j-idcY)*cellSizeY;
			centerCell.z = centerZ + (k-idcZ)*cellSizeZ;

			center = link->getCenter();
			//now add the line center--centerCell
			fprintf(fout,"%lf %lf %lf\n\t%lf %lf %lf\n\t",
				center.x, center.y, center.z, 
				centerCell.x, centerCell.y, centerCell.z);
			n_lines++;
		    }
		}
	    }
	}
    }
    
    fprintf(fout, "]\n\t}\n\tcoordIndex [\n\t");
    for(int i = 0; i<n_lines; i++) {
	fprintf(fout,"%d, %d, -1\n\t",2*i, 2*i+1);	
    }
    fprintf(fout, "]\n}\n}\n");

}
