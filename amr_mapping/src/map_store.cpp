/* -*- mode: c++; -*- */

/** @file map_store.cpp Implementation of the MapStore class.
 *
 *  The MapStore class provides grid-based storage.
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "map_store.h"
#include "map_store_beam.h"

#if defined(DEBUG_FILLCONE) || defined(DEBUG_LOAD)
#include <iostream>
using std::cout;
using std::endl;
#endif

using namespace mapstore;

MapStore::MapStore(int initSizeX, int initSizeY) : sizeX(initSizeX | 1), sizeY(initSizeY | 1), originX(sizeX/2), originY(sizeY/2) {
  maxIdx = sizeX*sizeY;
  data = new double[maxIdx];
  int i;
  for (i=0; i < maxIdx; i++)
    data[i] = 0.0;
}

MapStore::MapStore(const char *mapFileName) : sizeX(3), sizeY(3), originX(1), originY(1)  {
	maxIdx = 9;
	data = new double[maxIdx];
	bool res = loadMap(mapFileName);
	if (!res) {
		delete[] data;
		throw MapStoreError(MapStoreError::Format);
	}
}

MapStore::~MapStore() {
  delete[] data;
}

/** Compute index into data array.
 *
 *  Parameter x and y are in map coordinates.
 */
int MapStore::makeIdx(const int x, const int y) const {
  return (y+originY)*sizeX+(x+originX);
}

/** Convert x coordinate from map to array offset.
 *
 *  Parameter x is in map coordinates.  It will be converted into a
 *  column index in the data array.
 *
 *  This method returns the converted value in the input parameter x,
 *  whch is passed by reference.
 *
 *  The method returns as return value a flag, indicating if x is in
 *  its valid range.  Returnvalue will be false if x is outside its
 *  range.
 */
bool MapStore::convX(int &x) const {
  // -originX <= x < sizeX-originX
  // 0 <= (x+originX) < sizeX
  x += originX;
  return ((0 <= x) && (x < sizeX));
}

/** Convert y coordinate from map to array offset.
 *
 *  Parameter y is in map coordinates.  It will be converted into a
 *  column index in the data array.
 *
 *  This method returns the converted value in the input parameter y,
 *  whch is passed by reference.
 *
 *  The method returns as return value a flag, indicating if y is in
 *  its valid range.  Returnvalue will be false if y is outside its
 *  range.
 */
bool MapStore::convY(int &y) const {
  // -originY <= y < sizeY-originY
  // 0 <= (y+originY) < sizeY
  y += originY;
  return ((0 <= y) && (y < sizeY));
}


/** Convert and clip range from map coordinates into data array coordinates. 
 *
 *  This method takes a range of map coordinates, given as a point and
 *  size, and converts them into data array coordinates.  The size can
 *  be given as a positive or negative number, with negative values
 *  interpreted as the inclusive range from (x - |dx| + 1) to x, and
 *  positive values interpreted as the inclusive range from x to (x +
 *  dx -1).
 *
 *  If the range exceeds the maps area, it will be clipped to the map
 *  area.  The method returns in x the lowest data array coordinate
 *  (ie. the starting column index) of the range and in dx the size of
 *  the (possibly clipped) range.
 *
 *  If the range is completely outside the map area, then x is
 *  undefined after return and dx is set to zero.
 */
void MapStore::convXRange(int &x, int &dx) const {
  int xStart = x + originX;
  int xEnd = xStart + dx;

  if (xStart < 0)
    xStart = 0;
  if (xStart > sizeX)
    xStart = sizeX;

  if (xEnd > sizeX)
    xEnd = sizeX;
  if (xEnd < 0)
    xEnd = 0;

  dx = xEnd - xStart;
  if (dx < 0)
    dx = 0;

  x = xStart;
}

/** Convert and clip range from map coordinates into data array coordinates. 
 *
 *  For details see convXRange();
 */
void MapStore::convYRange(int &y, int &dy) const {
  int yStart = y + originY;
  int yEnd = yStart + dy;

  if (yStart < 0)
    yStart = 0;
  if (yStart > sizeY)
    yStart = sizeY;

  if (yEnd > sizeY)
    yEnd = sizeY;
  if (yEnd < 0)
    yEnd = 0;

  dy = yEnd - yStart;
  if (dy < 0)
    dy = 0;

  y = yStart;
}


bool MapStore::grow(int x, int y, int size) {
  int newMinX = minX();
  int xExpand = 0;
  int delta =  (x-size) - newMinX;
  int xShift = 0;
  if (delta < 0) {
    xExpand = -delta;
    xShift = xExpand;
    newMinX = x-size;
  }

  int newMaxX = maxX();
  delta =  (x+size) - newMaxX;
  if (delta > 0) {
    xExpand += delta;
    newMaxX = x+size;
  }


  int newMinY = minY();
  int yExpand = 0;
  int yShift = 0;
  delta =  (y-size) - newMinY;
  if (delta < 0) {
    yExpand = -delta;
    yShift = yExpand;
    newMinY = y-size;
  }

  int newMaxY = maxY();
  delta =  (y+size) - newMaxY;
  if (delta > 0) {
    yExpand += delta;
    newMaxY = y+size;
  }

  /* newMax - newMin must be even, otherwise expand one more.  This is
   * the case when Expand id even, because initially the difference is
   * even. */
  if (xExpand > 0) {
    if (xExpand & 1) {
      xExpand += 1;
      newMaxX += 1;
    }
  }

  if (yExpand > 0) {
    if (yExpand & 1) {
      yExpand += 1;
      newMaxY += 1;
    }
  }

  if (xExpand > 0 || yExpand > 0) {
    int newSizeX = sizeX+xExpand;
    int newSizeY = sizeY+yExpand;
    double *newdata = new double[newSizeX*newSizeY];
    int srcIdx = 0;
    int destIdx = 0;
    for (y=0; y < sizeY; y++) {
      destIdx = (y+yShift)*newSizeX + xShift;
      for (x=0; x < sizeX; x++) {
	newdata[destIdx++] = data[srcIdx++];
      }
    }
    delete[] data;
    data = newdata;
    maxIdx = newSizeX*newSizeY;
    sizeX = newSizeX;
    sizeY = newSizeY;
    originX += xShift;
    originY += yShift;
  }
  return true;
}

void MapStore::set(int x, int y, double val)  {
  if (! (isInX(x) && isInY(y)) )
    throw MapStoreError(MapStoreError::Range);

  data[makeIdx(x,y)] = val;
}

double MapStore::get(int x, int y) const  {
  if (! (isInX(x) && isInY(y)) )
    throw MapStoreError(MapStoreError::Range);

  return data[makeIdx(x,y)];
}


MapStoreTrace MapStore::trace(double x, double y, double dir, double length) const  {
  if (! (isInX(lround(x)) && isInY(lround(y))) )
    throw MapStoreError(MapStoreError::Range);

  MapStoreBeam beamInfo(x, y, dir, length);

  MapStoreTrace result;

  int xIdx, yIdx;
  beamInfo.getInitCell(xIdx, yIdx);
  if (length > 0) {
    do {
      result.add(xIdx, yIdx, data[makeIdx(xIdx, yIdx)]);
    } while (beamInfo.nextCell(xIdx, yIdx));
  } else {
    do {
      result.add(xIdx, yIdx, data[makeIdx(xIdx, yIdx)]);
      beamInfo.nextCell(xIdx, yIdx);
    } while (isInX(xIdx) && isInY(yIdx));
  }
  return result;
}

void MapStore::fillRect(int x, int y, int dx, int dy, double val)  {
  if (dx < 0) {
    x += dx;
    x += 1;
    dx = -dx;
  }
  if (dx < 0) {
    y += dy;
    y += 1;
    dy = -dy;
  }
  convXRange(x, dx);
  convYRange(y, dy);
  if (dx == 0 || dy == 0) {
    throw MapStoreError(MapStoreError::Range);
  }

  internalFillRect(x, y, dx, dy, val);
}

void MapStore::eraseRect(int x, int y, int dx, int dy)  {
  fillRect(x, y, dx, dy, 0);
}

void MapStore::fillCone(double x, double y, double dir, double width, double len, double val)  {
  if (! (isInX(lround(x)) && isInY(lround(y))) )
    throw MapStoreError(MapStoreError::Range);

  MapStoreBeam beamInfoCenter(x, y, dir, len);
  MapStoreBeam beamInfoLeft(x, y, dir+width/2, len);
  MapStoreBeam beamInfoRight(x, y, dir-width/2, len);

  int xIdxCenter,yIdxCenter;
  int xIdxLeft,yIdxLeft;
  int xIdxRight,yIdxRight;

  beamInfoCenter.getInitCell(xIdxCenter, yIdxCenter);
  beamInfoLeft.getInitCell(xIdxLeft, yIdxLeft);
  beamInfoRight.getInitCell(xIdxRight, yIdxRight);

  MapStoreBeam beamInfoLeftFill(xIdxCenter, yIdxCenter, xIdxLeft, yIdxLeft);
  MapStoreBeam beamInfoRightFill(xIdxCenter, yIdxCenter, xIdxRight, yIdxRight);

  int idx;
  do {
#ifdef DEBUG_FILLCONE
    {
      int tx,ty;
      beamInfoCenter.getLastCell(tx, ty);
      cout << "Cone: advance center beam (" << tx << ", " << ty << ") len " << beamInfoCenter.getLen() << " --> ";
    }
#endif
    beamInfoCenter.nextCell(xIdxCenter, yIdxCenter);
#ifdef DEBUG_FILLCONE
    {
      cout << "(" << xIdxCenter << ", " << yIdxCenter << ") len " << beamInfoCenter.getLen() << endl;
      cout << "Cone: fill left to center 1" << endl;
    }
#endif
    beamInfoLeftFill.reInit(xIdxLeft, yIdxLeft, xIdxCenter, yIdxCenter);
    do {
      idx = makeIdx(xIdxLeft, yIdxLeft);
      if ( (0 <= idx) && (idx < maxIdx) )
	data[idx] = val;
    } while (beamInfoLeftFill.nextCell(xIdxLeft, yIdxLeft));

    while (beamInfoLeft.getLen() < beamInfoCenter.getLen()) {
#ifdef DEBUG_FILLCONE
      {
	int tx,ty;
	beamInfoLeft.getLastCell(tx, ty);
	cout << "Cone: advance left border (" << tx << ", " << ty << ") len " << beamInfoLeft.getLen() << " --> ";
      }
#endif
      beamInfoLeft.nextCell(xIdxLeft, yIdxLeft);
#ifdef DEBUG_FILLCONE
      {
	cout << "(" << xIdxLeft << ", " << yIdxLeft << ") len " << beamInfoLeft.getLen() << endl;
	cout << "Cone: fill left to center 2" << endl;
      }
#endif
      beamInfoLeftFill.reInit(xIdxLeft, yIdxLeft, xIdxCenter, yIdxCenter);
      do {
	idx = makeIdx(xIdxLeft, yIdxLeft);
	if ( (0 <= idx) && (idx < maxIdx) )
	  data[idx] = val;
      } while (beamInfoLeftFill.nextCell(xIdxLeft, yIdxLeft));
    }
    beamInfoLeft.getLastCell(xIdxLeft, yIdxLeft);

#ifdef DEBUG_FILLCONE
    cout << "Cone: fill right to center 1" << endl;
#endif
    beamInfoRightFill.reInit(xIdxRight, yIdxRight, xIdxCenter, yIdxCenter);
    do {
      idx = makeIdx(xIdxRight, yIdxRight);
      if ( (0 <= idx) && (idx < maxIdx) )
	data[idx] = val;
    } while (beamInfoRightFill.nextCell(xIdxRight, yIdxRight));

    while (beamInfoRight.getLen() < beamInfoCenter.getLen()) {
#ifdef DEBUG_FILLCONE
      {
	int tx,ty;
	beamInfoRight.getLastCell(tx, ty);
	cout << "Cone: advance right border (" << tx << ", " << ty << ") len " << beamInfoRight.getLen() << " --> ";
      }
#endif
      beamInfoRight.nextCell(xIdxRight, yIdxRight);
#ifdef DEBUG_FILLCONE
      {
	cout << "(" << xIdxRight << ", " << yIdxRight << ") len " << beamInfoRight.getLen() << endl;
	cout << "Cone: fill right to center 2" << endl;
      }
#endif
      beamInfoRightFill.reInit(xIdxRight, yIdxRight, xIdxCenter, yIdxCenter);
      do {
	idx = makeIdx(xIdxRight, yIdxRight);
	if ( (0 <= idx) && (idx < maxIdx) )
	  data[idx] = val;
      } while (beamInfoRightFill.nextCell(xIdxRight, yIdxRight));
    }
    beamInfoRight.getLastCell(xIdxRight, yIdxRight);

  } while (beamInfoCenter.getLen() < len);
}


void MapStore::internalFillRect(int x, int y, int dx, int dy, double val) {
  int destIdx = 0;
  int yi,xi;
  for (yi = 0; yi < dy; yi++) {
    destIdx = (y + yi)*sizeX + x;
    for (xi = 0; xi < dx; xi++) {
	data[destIdx++] = val;
    }
  }
}


void MapStore::dumpGP(FILE *fh, const char* title) const {
  dumpDoubleToGP(fh, title, data, sizeX, sizeY, true);
}

void MapStore::startGPMultiplot(FILE *fh, int numMaps) {
  switch (numMaps) {
  case 2:
    fprintf(fh, "set multiplot layout 2,1\n");
    break;
  case 3:
    fprintf(fh, "set multiplot layout 2,2\n");
    break;
  case 4:
    fprintf(fh, "set multiplot layout 2,2\n");
    break;
  default:
    fprintf(stderr, "Warning: setGPMultiplot() called with illegal numMaps parameter: %d not within [2:4]\n", numMaps);
    break;
  }
}

void MapStore::commitGPMultiplot(FILE *fh) {
  fprintf(fh, "unset multiplot\n");
  fflush(fh);
}

FILE *MapStore::startGP(void) {
  return popen("gnuplot >gnuplot.log 2>gnuplot.err", "we");
}

void MapStore::stopGP(FILE *fh) {
  if (fh)
    pclose(fh);
}


void MapStore::dumpDoubleToGP(FILE *fh, const char *title, double *data, int sizeX, int sizeY, bool rowMajor) {
  fprintf(fh, "set title '");
  fprintf(fh, "%s", title);
  fprintf(fh, "'\nset pm3d map\nsplot '-' matrix\n");
  int yrowOff = 0;
  int yrow;
  int xcol;
  if (rowMajor) {
    for (yrow = 0; yrow < sizeY; yrow++) {
      for (xcol = 0; xcol < sizeX; xcol++) {
	fprintf(fh, " %.4f", data[xcol+yrowOff]);
      }
      yrowOff += sizeX;
      fprintf(fh, "\n");
    }
  } else {
    for (yrow = 0; yrow < sizeY; yrow++) {
      for (xcol = 0; xcol < sizeX; xcol++) {
	fprintf(fh, " %.4f", data[xcol*sizeY+yrow]);
      }
      fprintf(fh, "\n");
    }
  }
  fprintf(fh, "e\ne\n\n\n");
  fflush(fh);
}

bool MapStore::loadMap(const char *mapFileName) {
	FILE *fh = fopen(mapFileName, "r");
	if (fh == 0)
		return false;
	bool res = loadMap(fh);
	fclose(fh);
	return res;
}

bool MapStore::loadMap(FILE *mapFH) {
	enum loadStages {
		lsSearchHeader,
		lsMayFoundHeader,
		lsInHeader,
		lsSkipGP,
		lsFoundData,
		lsReadData,
		lsDataOk,
		lsDataFail
	};
	
	bool headerPresent = false;
//	bool headerComplete = false;
	
	// this could be put in struct, but since it is needed only once here, I used simple variables.
	bool hasSize = false;
	int mhSizeX;
	int mhSizeY;
	bool hasOrigin = false;
	int mhOriginX;
	int mhOriginY;
        bool hasResolution = false;
        double mhResolution;

	// progress through the file's sections
	loadStages stage = lsSearchHeader;
	bool badFile = false;  // flag, set to true if format error detected.
	bool readNext = true; // flag, true if a new line should be read.
	
	char *buffer = new char[8192];
	char *pos = buffer;
	int rowOffset = 0; // index into data array: start of current row.
	int idx = 0; // character index into current line
	int curRow = 0; // currently processed row
	
	while ( (!feof(mapFH)) && (stage != lsDataOk) && (stage != lsDataFail) && (badFile == false) ) {
		if (readNext)
			pos = fgets(buffer, 8190, mapFH);
			
		if (pos == 0)
			break; // either error or end-of-file  further check below
			
		if (pos[0] == '#')
			continue; // comment line, skip it.
			
		switch (stage) {
		case lsSearchHeader:
			/* the header may have comments or empty lines before it. 
			 *  So this stage ends at the first non-empty line. */
			 idx = strspn(pos, " \t"); // first char not in list
			 if (pos[idx] == '\n') {
				readNext = true;  // empty line, skip it.
			} else if (pos[0] == ' ') {
				stage = lsFoundData;  // starts with space, possible data row
				readNext = false; // process data next round
			} else {
				stage = lsMayFoundHeader; // everything else is either a header line or visualization data.
				readNext = false; // process data next round
			}
			break;

		case lsMayFoundHeader:
			idx = strcspn(pos, ":");
			if (idx > 0) {
				// colon present, check text to its left
				if (!strncmp(pos, "MapSize",idx)) {
					// found header "MapSize"
					stage = lsInHeader;
				} else if (!strncmp(pos, "MapRes",idx)) {
					// found header
					stage = lsInHeader;
				} else if (!strncmp(pos, "MapType",idx)) {
					// found header
					stage = lsInHeader;
				} else if (!strncmp(pos, "MapOrigin",idx)) {
					// found header
					stage = lsInHeader;
				} else {
					// not a recognized header
					stage =lsSkipGP;
				}
			} else {
				// no colon, so not a header line
				stage = lsSkipGP;
			}
			readNext = false; // process data next round
			break;

		case lsInHeader:
			idx = strcspn(pos, ":");
			if (idx > 0) {
				// colon present, check text to its left
				if (!strncmp(pos, "MapSize", idx)) {
					// found header "MapSize"
					if (hasSize) {
						// two size headers -> format error
						badFile = true;
					} else {
						int conv = sscanf(&pos[idx+1], " %d %d", &mhSizeX, &mhSizeY);
						if (conv != 2) {
							badFile = true;
						} else {
							hasSize = true;
						}
					}
					readNext = true; // done, next line
				} else if (!strncmp(pos, "MapRes", idx)) {
					// found header "MapRes"
					if (hasResolution) {
						// two resolution headers -> format error
						badFile = true;
					} else {
						int conv = sscanf(&pos[idx+1], " %lf", &mhResolution);
						if (conv != 1) {
							badFile = true;
						} else {
							hasResolution = true;
						}
					}
					readNext = true; // done, next line
				} else if (!strncmp(pos, "MapType", idx)) {
					// found header, do nothing
					readNext = true; // done, next line
				} else if (!strncmp(pos, "MapOrigin", idx)) {
					// found header
					if (hasOrigin) {
						// two origin headers -> format error
						badFile = true;
					} else {
						int conv = sscanf(&pos[idx+1], " %d %d", &mhOriginX, &mhOriginY);
						if (conv != 2) {
							badFile = true;
						} else {
							hasOrigin = true;
						}
					}
					readNext = true; // done, next line
				} else {
					// not a recognized header
					stage =lsSkipGP;
//					headerComplete = true;
					readNext = false; // process data next round
				}
			} else {
				// no colon, so not a header line
				stage = lsSkipGP;
//				headerComplete = true;
				readNext = false; // process data next round
			}
			break;

		case lsSkipGP:
			/*  the visualization info may have comments or empty lines
			 *  in or before it. 
			 *  So this stage ends at the first line being a data line. */
			 idx = strspn(pos, " \t"); // first char not in list
			 if (pos[idx] == '\n') {
				readNext = true;  // empty line, skip it.
			} else if (pos[0] == ' ') {
				stage = lsFoundData;  // starts with space, possible data row
				readNext = false; // process data next round
			} else {
				// everything else is either a header line or visualization data.
				// do nothing
			}
			break;

		case lsFoundData:
			// first data row.  check if we have all needed information to read it
			headerPresent = hasSize || hasOrigin;
			if (headerPresent && !hasSize) {
				// size is mandatory if a header exists.
				badFile = true;
				break;
			}
			if (headerPresent) {
				// header present, reset map.
				delete[] data;
				sizeX = mhSizeX | 1;
				sizeY = mhSizeY | 1;
				maxIdx = sizeX*sizeY;
				data = new double[maxIdx];
				if (hasOrigin) {
					// mhOrigin is the cell coordinate that addresses the center of the map
					originX = sizeX/2 - mhOriginX;  // sizeX/2 = originX +mhOriginX
					originY = sizeY/2 - mhOriginY;
				} else {
					// 0;0 is at map's center
					originX = sizeX/2;
					originY = sizeY/2;
				}
				if (hasResolution) {
					resolution = mhResolution;
				} else {
					resolution = 1.0;
				}
			}

			stage = lsReadData;
			readNext = false; // process data next round
			break;
			
		case lsReadData:
			if (pos[0] == 'e') {
				stage = lsDataOk;
			} else if (pos[0] == ' ') {
				// data line
				char *pnt = pos;
				char *pnt2 = 0;
				int i;
#ifdef DEBUG_LOAD
				std::cout << "reading row " << curRow << " at offset " << rowOffset << std::endl;
#endif
				for (i = 0; i < mhSizeX; i++) {
					data[rowOffset+i] = strtod(pnt, &pnt2);
					if (pnt2 == pnt) {
						// conversion error
						badFile = true;
						stage = lsDataFail;
						break;
					}
					pnt = pnt2;
				}
				// here pnt2 must be at the end of the line and i must be mhSizeX
				idx = strspn(pnt2, " \t");
				if (pnt2[idx] != '\n') {
					// garbage after data
					badFile = true;
					stage = lsDataFail;
				}
				if (i != mhSizeX) {
					// short line
					badFile = true;
					stage = lsDataFail;
				}
				rowOffset += sizeX;
				curRow += 1;
				if (curRow == mhSizeY)
					stage = lsDataOk;
					
				readNext = true;
			} else {
				// format error
				badFile = true;
				stage = lsDataFail;
			}
			break;
			
		case lsDataOk:
#ifdef DEBUG_LOAD
			std::cout << "lsDataOk reached" << std::endl;
			break;
#endif
			// fall through
		case lsDataFail:
			// do nothing, while () will terminate.
#ifdef DEBUG_LOAD
			std::cout << "lsDataFail reached" << std::endl;
#endif
			break;
		}
	} // end of while
	delete[] buffer;
	if (stage == lsDataFail)
		throw MapStoreError(MapStoreError::Format);
#ifdef DEBUG_LOAD
	std::cout << "badFile is " << ((badFile == true) ? "true" : "false") << std::endl; 
#endif
	return(!badFile);
}

MapStoreTrace::MapStoreTrace() {}

void MapStoreTrace::add(int x, int y, double val) {
  cells.push_back(MapStoreCell(x, y, val));
}

void MapStoreTrace::add(const MapStoreCell &c) {
  cells.push_back(c);
}
