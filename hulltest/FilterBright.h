#ifndef PIXELCOUNT_H
#define PIXELCOUNT_H

void FilterBrightRects(vector<PolyVertices> &list);
bool FilterBrightSub(PolyVertices poly);
void GetBounds(int x0, int x1, int y0, int y1, vector<CvPoint> &bounds);

#endif
