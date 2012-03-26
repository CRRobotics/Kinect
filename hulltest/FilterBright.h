#ifndef PIXELCOUNT_H
#define PIXELCOUNT_H

void FilterBrightRects(vector<PolyVertices> &list, IplImage *img);
bool FilterBrightSub(PolyVertices poly, IplImage *img);
void GetBounds(int x0, int x1, int y0, int y1, vector<int> &bounds);

#endif
