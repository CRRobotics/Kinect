void PolyVertices::invalidate()	{ center.x = center.y = 0; }

bool PolyVertices::isValid() { return (center.x != 0 && center.y != 0); }

bool PolyVertices::isMalformed()
{
	return false;
}

// Compare centers
bool operator== (const PolyVertices& arg1, const PolyVertices& arg2) { return (arg1.center.x == arg2.center.x && arg1.center.y == arg2.center.y); }

/*
 * Converts n-gons to 4-gons.
 * qKey order is, CCW from top right, 3->2->0->1.
 */
void polyToQuad(CvSeq* sequence, PolyVertices *poly, IplImage* img )
{
	// Sequence is the hull.
	// poly->points is the array that stores the points of the rectangle.
	// img is the image we are drawing lines and such on. Not strictly necessary.
	#ifdef DEBUG_POLY
	printf("Sequence: %d\n", sequence->total);
	#endif

	/*CALCULATE CENTER*/
	// int center[2] = {0}; //center point of the poly x, y
	int extremes[4]; //minX, maxX, minY, maxY
	extremes[0] = ((CvPoint*)cvGetSeqElem(sequence, 0))->x;
	extremes[1] = extremes[0];
	extremes[2] = ((CvPoint*)cvGetSeqElem(sequence, 0))->y;
	extremes[3] = extremes[2];


	CvSeqReader seqReader;
	cvStartReadSeq(sequence, &seqReader, 0);
	for(int i = 0; i < sequence->total; i++)
	{
		CvPoint* point = (CvPoint*) seqReader.ptr;

		if(point->x < extremes[0]) extremes[0] = point->x;
		if(point->x > extremes[1]) extremes[1] = point->x;

		if(point->y < extremes[2]) extremes[2] = point->y;
		if(point->y > extremes[3]) extremes[3] = point->y;
		CV_NEXT_SEQ_ELEM(seqReader.seq->elem_size, seqReader);
	}
	poly->center.x = (extremes[0] + extremes[1])/2;
	poly->center.y = (extremes[2] + extremes[3])/2;

	cvCircle( img, poly->center, 2, CV_RGB(255,0,255), -1, 8, 0 );
	#ifdef DEBUG_POLY
	printf("Calculated Center (%d): %i, %i\n", sequence->total, poly->center.x, poly->center.y);
	#endif

	/*CALCULATE DISTANCES FROM CENTER AND CALCULATE MAX DIST IN EACH QUADRANT*/
	double distances[4] = {0, 0, 0, 0};
	double distance; //current point's distance

	int qKey = 0;

	CvSeqReader seqReader2;
	cvStartReadSeq(sequence, &seqReader2, 0);
	for(int i = 0; i < sequence->total; i++)
	{
		CvPoint* point = (CvPoint*) seqReader2.ptr;
		distance = pow((double)point->x - poly->center.x, 2) + pow((double)point->y - poly->center.y, 2);
		qKey = ((point->x > poly->center.x ? 1 : 0) + (point->y < poly->center.y ? 2 : 0));

		/* FILL POINTS */
		if(distance > distances[qKey])
		{
			poly->points[qKey] = point;
			distances[qKey] = distance;
		}
		CV_NEXT_SEQ_ELEM(seqReader2.seq->elem_size, seqReader2);
	}

	if(!(poly->points[0] == NULL || poly->points[1] == NULL || poly->points[2] == NULL || poly->points[3] == NULL))
	{
		/* FILL poly->dist */
		poly->dist = pow((double)poly->points[0]->x - poly->center.x, 2) + pow((double)poly->points[0]->y - poly->center.y, 2);

		#ifdef DEBUG_POLY
		printf("Dist to 0: %d\n", poly->dist);
		#endif

		// Draw vertices (purple circles)
		cvCircle( img, *poly->points[0], 3, CV_RGB(255,0,255), -1, 8, 0 );
		cvCircle( img, *poly->points[1], 3, CV_RGB(255,0,255), -1, 8, 0 );
		cvCircle( img, *poly->points[2], 3, CV_RGB(255,0,255), -1, 8, 0 );
		cvCircle( img, *poly->points[3], 3, CV_RGB(255,0,255), -1, 8, 0 );
	}
}

// Return whether we want to delete poly. Subroutine of FilterInnerRects.
bool FilterSub(vector<PolyVertices> &list, PolyVertices poly)
{
	for (int i = 0; i < list.size(); i++)
	{
		double result = pow((double)list[i].center.x - (double)poly.center.x, 2) + pow((double)list[i].center.y - (double)poly.center.y, 2);
		if ((result < 50 * 50) && (list[i].dist != poly.dist))
			if (list[i].dist > poly.dist)
				return true;
	}
	return false;
}

// Remove rectangles enclosed by others.
void FilterInnerRects(vector<PolyVertices> &list)
{
	vector<PolyVertices>::iterator p = list.begin();
	while (p < list.end()) 
	{
		if (FilterSub(list, *p))
			p = list.erase(p);
		else
			++p;
	}
}

// The word "must" in the following function should be interpreted to mean "if we are not missing more than 
// however many rectangles we think we are missing." There is really no provision yet for missing >1, but it 
// shouldn't be too much of an issue.
void SortRects(vector<PolyVertices> &list)
{
	// Fill with invalid structs to ensure no segfaults.
	// TODO: if we decide that missing >1 is not worth coding, we could return here if 4 - list.size() is >1.
	#ifdef DEBUG_SORT
	printf("Empty structs pushed: %d\n", 4 - list.size());
	#endif
	for (int i = list.size(); i < 4; i++)
	{
		PolyVertices temp;
		list.push_back(temp);
	}

	PolyVertices top, left, right, bottom;
	top = left = right = bottom = list[0];

	/* FILL WITH PRELIM. DATA */
	for (vector<PolyVertices>::iterator p = list.begin(); p < list.end(); p++)
	{
		if ((*p).isValid())
		{
			if ((*p).center.y < top.center.y)
				top = *p;	
			if ((*p).center.y > bottom.center.y)
				bottom = *p;
			if ((*p).center.x < left.center.x)
				left = *p;
			if ((*p).center.x > right.center.x)
				right = *p;
		}
	}
	
 	/* FIND WHICH RECTANGLE IS MISSING */ 
 	if (top == left || top == right)
 	{
 		// CASE: top and left are ambiguous
 		if (abs(top.center.x - left.center.x) < 50)
 		{
 			// right and bottom must be correct
 			list[2] = right;
 			list[3] = bottom;
 
 			if (abs(top.center.x - bottom.center.x) < 50) // both are top
 			{
 				list[0] = top;
 				list[1].invalidate();
 			}
 			else // both are left
 			{
 				list[0].invalidate();
 				list[1] = left;
 			}
 		}
 		// CASE: top and right are ambiguous
 		else if (abs(top.center.x - right.center.x) < 50)
 		{
 			// left and bottom must be correct
 			list[1] = left;
 			list[3] = bottom;
 			
 			if (abs(top.center.x - bottom.center.x) < 50) // both are top
 			{
 				list[0] = top;
 				list[2].invalidate();
 			}
 			else // both are right
 			{
 				list[0].invalidate();
 				list[2] = right;
 			}
 		}
	}	

	else if (bottom == left || bottom == right)
	{
		// CASE: bottom and left are ambiguous
 		if (abs(bottom.center.x - left.center.x) < 50)
 		{
 			// top and right must be correct
 			list[0] = top;
 			list[2] = right;
 
 			if (abs(top.center.x - bottom.center.x) < 50) // both are bottom
 			{
 				list[1].invalidate();
 				list[3] = bottom;
 			}
 			else // both are left
 			{
 				list[1] = left;
 				list[3].invalidate();
 			}
 		}
 		// CASE: bottom and right are ambiguous
 		else if (abs(bottom.center.x - right.center.x) < 50)
 		{
 			// top and left must be correct
 			list[0] = top;
 			list[1] = left;
 			
 			if (abs(top.center.x - bottom.center.x) < 50) // both are bottom
 			{
 				list[2].invalidate();
 				list[3] = bottom;
 			}
 			else // both are right
 			{
 				list[2] = right;
 				list[3].invalidate();
 			}
 		}
 	}

	else
	{
		list[0] = top;
		list[1] = left;
		list[2] = right;
		list[3] = bottom;
	}

	#ifdef DEBUG_SORT
 	for (int i = 0; i < 4; i++)
 	{
 		printf("[%d]: (%d, %d)\n", i, list[i].center.x, list[i].center.y);
 	}
	#endif
}
