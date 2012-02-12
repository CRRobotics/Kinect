#ifndef ROBOTPROC_H
#define ROBOTPROC_H

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

bool operator== ( const PolyVertices& arg1, const PolyVertices& arg2 );

void polyToQuad ( CvSeq* sequence, PolyVertices *poly, IplImage* img );

bool FilterSub ( vector<PolyVertices> &list, PolyVertices poly );

void FilterInnerRects ( vector<PolyVertices> &list );

void SortRects ( vector<PolyVertices> &list );

