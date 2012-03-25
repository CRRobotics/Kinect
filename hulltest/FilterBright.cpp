//#define DEBUG_BRITE

/*
 * Filters out rects mostly filled with white.
 * This removes things like lights, which are solid blocks of white.
 * Code is essentially copy of FilterInnerRects().
 */
void FilterBrightRects(vector<PolyVertices> &list)
{
	vector<PolyVertices>::iterator p = list.begin();
	while (p < list.end()) 
	{
		if (FilterBrightSub(*p))
			p = list.erase(p);
		else
			++p;
	}
}

/*
 * Decides if a rect is filled enough to be filtered out.
 * Subroutine of FilterBrightRects().
 */
bool FilterBrightSub(PolyVertices poly)
{
	vector<CvPoint> leftBounds;
	vector<CvPoint> rightBounds;
	CvPoint top, left, right, bottom;
	top = left = right = bottom = poly.points[0];
	for (int i = 0; i < 4; i++)
	{
		if (points[i].y < top.y)
			top = points[i];
		if (points[i].y > bottom.y)
			bottom = points[i];
		if (points[i].x < bottom.x)
			left = points[i];
		if (points[i].x > bottom.x)
			right = points[i];
	}

	/*
	 * TODO: find which of the four sides constitute the "left" bounds and
	 * which sides constitute the "right" bounds must be found before we can
	 * start calling GetBounds().
	 * As of yet, I have no ideas on how to decide this; it might just be a
	 * huge pile of cases, but I'd rather use a more elegant solution.
	 *
	 * Following is incorrect code illustrating how GetBounds() would be
	 * called conceptually.

	GetBounds(top.x, left.x, top.y, left.y, leftBounds);
	GetBounds(left.x, bottom.x, left.y, bottom.y, leftBounds);
	GetBounds(top.x, right.x, top.y, right.y, rightBounds);
	GetBounds(right.x, bottom.x, right.y, bottom.y, rightBounds);

	 */

	// Avoid division by zero in return
	int white = 1;
	int black = 1;
	// top.y is the baseline, and is conceptually 0 for purposes of getting
	// the appropriate points from leftBound and rightBound.
	int yOffset = top.y;
	for (int y = 0; y < bottom.y - yOffset; y++)
	{
		for (int x = leftBound[y]; x < rightBound[y]; x++)
		{
			/* TODO: ACCESS EACH PIXEL AND GET VALUE */
			/*
			 * Pseudocode:
			 * if pixel at (y + yOffset, x) is white
			 *   white++
			 * else
			 *   black++
			 */
		}
	}

	/*
	 * Return true if proportion of white greatly exceeds black; a ratio of
	 * something like 9:1 (maybe even higher) sounds about right; decide on
	 * this later.
	 */
	return (double)(white/black) > 0.9;
}

/*
 * Implementation of Bresenham's line algorithm.
 * Gets all points in a line, for use in making left/right boundaries on each
 * line for FilterBrightRects to count all pixels in a rect.
 * This implementation performs push_back() only and also processes points in
 * the order in which they appear, top to bottom.
 */
void GetBounds(int x0, int x1, int y0, int y1, vector<CvPoint> &bounds)
{
	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep)
	{
		swap(x0, y0);
		swap(x1, y1);
	}
	int deltax = abs(x1 - x0);
	int deltay = abs(y1 - y0);
	int error = deltax / 2;
	int ystep;
	int y = y0;
	int inc;

	inc = (x0 < x1) ? 1 : -1;
	ystep = (y0 < y1) ? 1 : -1;

	for (int x = x0; x < x1; x += inc)
	{
		bounds.push_back(steep ? CvPoint(y,x) : CvPoint(x,y));
		error = error - deltay;
		if (error < 0)
		{
			y += ystep;
			error += deltax;
		}
	}
}

void swap(int &a, int &b)
{
	int temp = a;
	a = b;
	b = temp;
}
