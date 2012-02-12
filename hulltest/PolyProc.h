#ifndef POLYPROC_H
#define POLYPROC_H

class PolyVertices
{
public:
	CvPoint* points[4];
	CvPoint center; 
	int dist;

	PolyVertices();
	void invalidate();
	bool isValid();
	bool isMalformed();
};

void polyToQuad(CvSeq* sequence, PolyVertices *poly, IplImage* img, bool display);
bool FilterSub(vector<PolyVertices> &list, PolyVertices poly);
void FilterInnerRects(vector<PolyVertices> &list);
void SortRects(vector<PolyVertices> &list);

#endif
