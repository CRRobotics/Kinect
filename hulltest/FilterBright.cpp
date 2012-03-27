#define DEBUG_BRITE

#include <cv.h>

#include "FilterBright.h"

using namespace std;

/*
 * Filters out rects mostly filled with white.
 * This removes things like lights, which are near-solid blocks of white.
 * Code is essentially copy of FilterInnerRects().
 */
void FilterBrightRects(vector<PolyVertices> &list, IplImage *img)
{
	vector<PolyVertices>::iterator p = list.begin();
	while (p < list.end()) 
	{
		if (FilterBrightSub(*p, img))
			p = list.erase(p);
		else
			++p;
	}
}

/*
 * Decides if a rect is filled enough to be filtered out.
 * Subroutine of FilterBrightRects().
 */
bool FilterBrightSub(const PolyVertices &poly, IplImage *img)
{
	vector<int> leftBounds;
	vector<int> rightBounds;
	// Get copies; TR is top right point, and so on
	CvPoint TR, TL, BL, BR;
	TR = *(poly.points[3]);
	TL = *(poly.points[2]);
	BL = *(poly.points[0]);
	BR = *(poly.points[1]);

	/* TODO: Unsure whether the logic in the following cases is sound. */

	/* TODO: not yet handling special cases of horizontal top or bottom
	 * sides. I think it could be handled by having getBounds() push 
	 * only the left point to leftBounds and only the right point to 
	 * rightBounds. 
	 */

	// Range of rows to scan
	int topY;
	int botY;

	// TR is lower than TL:
	if (TR.y > TL.y)
	{
		topY = TL.y;
		// TL to TR is first right bound
		GetBounds(TL.x, TR.x, TL.y, TR.y, rightBounds);
		// TL to BL is first left bound
		GetBounds(TL.x, BL.x, TL.y, BL.y, leftBounds);

		// BR is lower than BL:
		if (BR.y > BL.y)
		{
			botY = BR.y;
			// Left edge is TL-BL-BR, right edge is TL-TR-BR
			GetBounds(TL.x, BL.x, TL.y, BL.y, leftBounds);
			GetBounds(TR.x, BR.x, TR.y, BR.y, rightBounds);
		}
		// BR is higher than BL:
		else if (BR.y < BL.y)
		{
			botY = BL.y;
			// Left edge is TL-BL, right edge is TL-TR-BR-BL
			GetBounds(TR.x, BR.x, TR.y, BR.y, rightBounds);
			GetBounds(BR.x, BL.x, BR.y, BL.y, rightBounds);
		}
		// BR is on same row as BL:
		else
		{
			assert(BR.y == BL.y && "TR.y not equal to TL.y despite check");
			botY = BL.y;
			// Left edge is TL-BL, right edge is TL-TR-BR
			GetBounds(TR.x, BR.x, TR.y, BR.y, rightBounds);
		}
	}
	// TR is higher than TL:
	else if (TR.y < TL.y)
	{
		topY = TR.y;
		// TR to TL is first left bound
		GetBounds(TR.x, TL.y, TR.y, TL.y, leftBounds);
		// TR to BR is first right bound
		GetBounds(TR.x, BR.x, TR.y, BR.y, rightBounds);

		// BR is lower than BL:
		if (BR.y > BL.y)
		{
			botY = BR.y;
			// Left edge is TR-TL-BL-BR, right edge is TR-BR
			GetBounds(TL.x, BL.x, TL.y, BL.y, leftBounds);
			GetBounds(BL.x, BR.x, BL.y, BR.y, leftBounds);
		}
		// BR is higher than BL:
		else if (BR.y < BL.y)
		{
			botY = BL.y;
			// Left edge is TR-TL-BL, right edge is TR-BR-BL
			GetBounds(TL.x, BL.x, TL.y, BL.y, leftBounds);
			GetBounds(BR.x, BL.x, BR.y, BR.y, rightBounds);
		}
		// BR on same row as BL:
		else
		{
			assert(BR.y == BL.y && "TR.y not equal to TL.y despite check");
			botY = BL.y;
			// Left edge is TR-TL-BL, right edge is TR-BR
			GetBounds(TL.x, BL.x, TL.y, BL.y, leftBounds);
		}
	}
	// TR is at equal level to TL:
	else
	{
		assert(TR.y == TL.y && "TR.y not equal to TL.y despite check");
		top.y = TR.y;
		// TL to BL is first left bound
		GetBounds(TL.x, BL.x, TL.y, BL.y, rightBounds);
		// TR to BR is first right bound
		GetBounds(TR.x, BR.x, TR.y, BR.y, rightBounds);

		// BR is lower than BL:
		if (BR.y > BL.y)
		{
			botY = BR.y;
			// Left edge is TL-BL-BR, right edge is TR-BR
			GetBounds(TL.x, BR.x, TL.y, BR.y, leftBounds);
		}
		// BR is higher than BL:
		else if (BR.y < BL.y)
		{
			botY = BL.y;
			// Left edge is TL-BL, right edge is TR-BR-BL
			GetBounds(BR.x, BL.x, BR.y, BL.y, rightBounds);
		}
		// BR on same row as BL:
		else
		{
			assert(BR.y == BL.y && "TR.y not equal to TL.y despite check");
			botY = BL.y;
			// Left edge is TL-BL, right edge is TR-BR
		}
	}

#ifdef DEBUG_BRITE
	printf("leftBounds.size(): %d\nrightBounds.size():%d\nbot.y - top.y: %d\n", leftBounds.size(), rightBounds.size(), bot.y - top.y);
#endif


	/* TODO: Verify that all memory-related use in this section is correct */

	// init to 1 to avoid possible division by zero in return
	int white = 1;
	int black = 1;
	// The indices for leftBound and rightBound must start at 0; the
	// pointers accessing the image must start at topY
	int yOffset = topY;
	for (int y = topY; y < botY; y++)
	{
		// Get pointer to beginning of row
		uchar *ptr = (uchar*)(img->imageData + (y * img->widthStep));
		// For all pixels on row from leftBounds to rightBounds
		for (int x = leftBounds[y - yOffset]; x < rightBounds[y - yOffset]; x++)
		{
			int pixel = ptr[x];
			if (pixel > 0)
				white++;
			else
				black++;
		}
	}

	/*
	 * Return true if proportion of white greatly exceeds black; a ratio of
	 * something like 9:1 (maybe even higher) sounds about right; decide on
	 * this later.
	 */
	return (double)(white/black) > 0.0;
}

/*
 * Implementation of Bresenham's line algorithm.
 * Gets leftmost x-point of a line on each row, for use in making left/right 
 * boundaries on each line for FilterBrightRects to count all pixels in a rect.
 * This implementation performs push_back() only and also processes points in
 */
void GetBounds(int x0, int x1, int y0, int y1, vector<CvPoint> &bounds)
{
	// Generalize for slope
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

	// Which direction to go in both x and y depending on slope
	inc = (x0 < x1) ? 1 : -1;
	ystep = (y0 < y1) ? 1 : -1;

	bool plotted = false;
	for (int x = x0; x < x1; x += inc)
	{
		// Only plot once per Y-line, and don't repeat on first point
		if (!plotted)
		{
			bounds.push_back(steep ? x : y);
			plotted = true;
		}
		error = error - deltay;
		if (error < 0)
		{
			y += ystep;
			error += deltax;
			plotted = false;
		}
	}

#ifdef DEBUG_BRITE
	printf("Delta y: %d -- Vector Size: %d", deltay, bounds.size());
#endif
}

void swap(int &a, int &b)
{
	int temp = a;
	a = b;
	b = temp;
}
