//#define DEBUG_BRITE

/*
 * Filters out rects mostly filled with white.
 * This removes things like lights, which are solid blocks of white.
 * Code is essentially copy of FilterInnerRects().
 */
void FilterBrightRects(PolyVertices* list)
{
	vector<PolyVertices>::iterator p = list.begin();
	while (p < list.end()) 
	{
		if (FilterBrightSub(list, *p))
			p = list.erase(p);
		else
			++p;
	}
}

/*
 * Decides if a rect is filled enough to be filtered out.
 * Subroutine of FilterBrightRects().
 */
void FilterBrightSub(PolyVertices* list, PolyVertices poly)
{
	vector<CvPoint> leftBounds;
	vector<CvPoint> rightBounds;
}

/*
 * Implementation of Bresenham's line algorithm.
 * Gets all points in a line, for use in making left/right boundaries on each
 * line for FilterBrightRects to count all pixels in a rect.
 */
void GetBounds(int x0, int x1, int y0, int y1, vector<CvPoint> &bounds)
{

}
