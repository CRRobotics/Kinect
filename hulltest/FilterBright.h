#ifndef PIXELCOUNT_H
#define PIXELCOUNT_H

void FilterBrightRects(PolyVertices* list);
void FilterBrightSub(PolyVertices* list, PolyVertices poly);
void GetBounds(int x0, int x1, int y0, int y1, vector<CvPoint> &bounds);

#endif
