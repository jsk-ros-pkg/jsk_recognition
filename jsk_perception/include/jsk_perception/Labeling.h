/* Labeling.h
 *
 * Copyright (c) 2010, IMURA Masataka.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef	__LABELING_H__
#define	__LABELING_H__

#include	<iostream>

#include	<algorithm>
#include	<list>
#include	<queue>

#define	CLEAR_DST_BUFFER		1
#define	CLEAR_ALL_DST_BUFFER	0
#define	CALC_CENTER_OF_GRAVITY	1

template<class SrcT, class DstT>
class Labeling {
public:

	// raster segment /////////////////////////////////////////////////////////

	class RasterSegment {
	private:
		int	left_x;
		int	right_x;
		int	y;
		SrcT	source_value;
	public:
		RasterSegment( const int n_left_x, const int n_right_x,
					   const int n_y, const SrcT n_source_value )
			: left_x( n_left_x ), right_x( n_right_x ), y( n_y ),
			  source_value( n_source_value )
		{
		}
	
		~RasterSegment()
		{
		}

		// get

		inline	int
		GetLeftX( void )	const
		{
			return left_x;
		}

		inline	int
		GetRightX( void )	const
		{
			return right_x;
		}
	
		inline	int
		GetY( void )	const
		{
			return y;
		}

		inline	SrcT
		GetSourceValue( void )	const
		{
			return source_value;
		}

		// get (short version)

		inline	int
		LeftX( void )	const
		{
			return left_x;
		}

		inline	int
		RightX( void )	const
		{
			return right_x;
		}
	
		inline	int
		Y( void )	const
		{
			return y;
		}

		inline	SrcT
		SourceValue( void )	const
		{
			return source_value;
		}

		friend	std::ostream&
		operator<<( std::ostream& s, RasterSegment& rs )
		{
			s << rs.LeftX() << " "
			  << rs.RightX() << " "
			  << rs.Y() << " "
			  << rs.SourceValue() << std::endl;

			return s;
		}
	};

	typedef	std::list<RasterSegment *>		RSPList;
	typedef	typename std::list<RasterSegment *>::iterator	RSPIterator;

	typedef	std::queue<RasterSegment *>	RSPQueue;

	// information about region ///////////////////////////////////////////////
	
	class	RegionInfo {
	
	private:
		int		num_of_pixels;
		float	center_x, center_y;
		int		size_x, size_y;
		int		min_x, min_y;
		int		max_x, max_y;
		SrcT	source_value;
		DstT	result;
		RSPList	raster_segment_list;
#if CALC_CENTER_OF_GRAVITY
		float	gravity_x, gravity_y;
#endif		
	public:
		// constructor and destructor
	
		RegionInfo()
		{
			raster_segment_list.clear();
		}
		
		~RegionInfo()
		{
			RSPIterator	rspi;
			for ( rspi = raster_segment_list.begin();
				  rspi != raster_segment_list.end(); rspi++ ) {
				RasterSegment	*rs = *rspi;
				delete rs;
			}
			raster_segment_list.erase( raster_segment_list.begin(),
									   raster_segment_list.end());
		}

		// a default copy constucter and an assignment operator
		// are suitable for this class.

		// declaration of functions

		// inline functions

		// set

		inline	void
		SetNumOfPixels( const int n_num_of_pixels )
		{
			num_of_pixels = n_num_of_pixels;
		}

		inline	void
		SetCenter( const float x, const float y )
		{
			center_x = x;
			center_y = y;
		}

		inline	void
		SetSize( const int x, const int y )
		{
			size_x = x;
			size_y = y;
		}

		inline	void
		SetMin( const int x, const int y )
		{
			min_x = x;
			min_y = y;
		}

		inline	void
		SetMax( const int x, const int y )
		{
			max_x = x;
			max_y = y;
		}

		inline	void
		SetMinMax( const int n_min_x, const int n_min_y,
				   const int n_max_x, const int n_max_y )
		{
			SetMin( n_min_x, n_min_y );
			SetMax( n_max_x, n_max_y );
			SetCenter(( n_min_x + n_max_x ) / 2.0f,
					  ( n_min_y + n_max_y ) / 2.0f );
			SetSize( n_max_x - n_min_x + 1, n_max_y - n_min_y + 1 );
		}

		inline	void
		SetCenterOfGravity( const float x, const float y )
		{
			gravity_x = x;
			gravity_y = y;
		}

		inline	void
		SetSourceValue( const SrcT n_source_value )
		{
			source_value = n_source_value;
		}

		inline	void
		SetResult( const DstT n_result )
		{		
			result = n_result;
		}

		// get

		inline	int
		GetNumOfPixels( void )	const
		{
			return num_of_pixels;
		}

		inline	void
		GetCenter( float& x, float& y )	const
		{
			x = center_x;
			y = center_y;
		}

		inline	void
		GetSize( int& x, int& y )	const
		{
			x = size_x;
			y = size_y;
		}

		inline	void
		GetMin( int& x, int& y )	const
		{
			x = min_x;
			y = min_y;
		}

		inline	void
		GetMax( int& x, int& y )	const
		{
			x = max_x;
			y = max_y;
		}

		inline	void
		GetCenterOfGravity( float& x, float& y )	const
		{
			x = gravity_x;
			y = gravity_y;
		}

		inline	SrcT
		GetSourceValue( void )	const
		{
			return source_value;
		}

		inline	DstT
		GetResult( void )	const
		{
			return result;
		}

		// list

		inline	RSPList&
		GetRasterSegmentList( void )
		{
			return raster_segment_list;
		}

		inline	void
		Push( RasterSegment *rs )
		{
			raster_segment_list.push_front( rs );
		}

		inline	void
		Pop( RasterSegment * & rs )
		{
			RSPIterator	rspi = raster_segment_list.begin();
			rs = *rspi;
			raster_segment_list.erase( rspi );
		}

		inline	int
		GetNumOfRasterSegments( void )
		{
			return raster_segment_list.size();
		}

		// operators

		friend	bool
		operator<( const RegionInfo& l, const RegionInfo& r )
		{
			bool	b = ( l.GetNumOfPixels() < r.GetNumOfPixels());
			return b;
		}

		friend	std::ostream&
		operator<<( std::ostream& s, RegionInfo& ri )
		{
			int	x, y;
			float	cx, cy;
			
			s << "num_of_pixels: " << ri.GetNumOfPixels() << std::endl;

			ri.GetCenter( cx, cy );
			s << "center: "        << cx << "," << cy << std::endl;

			ri.GetSize( x, y );
			s << "size:   "        << x << "," << y << std::endl;

			ri.GetMin( x, y );
			s << "min:    "        << x << "," << y << std::endl;

			ri.GetMax( x, y );
			s << "max:    "        << x << "," << y << std::endl;

#if CALC_CENTER_OF_GRAVITY
			ri.GetCenterOfGravity( cx, cy );
			s << "center_of_graivty: " << cx << "," << cy << std::endl;
#endif			
			
			s << "source_value: "  
			  << static_cast<int>( ri.GetSourceValue()) << std::endl
			  << "result: "
			  << static_cast<int>( ri.GetResult())      << std::endl;

			return s;
		}
	};

	typedef	std::list<RegionInfo *>		RIPList;
	typedef	typename std::list<RegionInfo *>::iterator	RIPIterator;

	typedef	std::vector<RegionInfo *>	RIPVector;

private:
	static	const	int	DEFAULT_REGION_SIZE_MIN = 10;
	
	SrcT	*src_frame;
	DstT	*dst_frame;
	int	width;
	int	height;
	int	total_num;

	RSPList	*raster_segment_list;
	int	num_of_raster_segments;

	RSPQueue	seed_queue;

	RIPList		region_info_list;
	int	num_of_regions;

	RIPVector	result_region_info;
	int	num_of_result_regions;

	// private functions

	void
	RegisterSegment( const int lx, const int rx,
					 const int y, const SrcT src_value )
	{
		RasterSegment	*rs = new RasterSegment( lx, rx, y, src_value );

		raster_segment_list[ y ].push_back( rs );
		num_of_raster_segments++;
	}

	void
	SearchNeighboringSegment( RasterSegment *rs_seed, const int dy )
	{
		RSPList	*rspl_p = &raster_segment_list[ rs_seed->Y() + dy ];
		RSPIterator	rspi;

		int	rs_seed_lx = rs_seed->LeftX();
		int	rs_seed_rx = rs_seed->RightX();
		int	rs_seed_source_value = rs_seed->SourceValue();

		rspi = rspl_p->begin();

#if 1
		if ( rspi == rspl_p->end()) {
			return;
		}

		while (( *rspi )->RightX() < rs_seed_lx ) {
			rspi++;
			if ( rspi == rspl_p->end()) {
				return;
			}
		}
		RasterSegment	*rs;
		while (( rs = *rspi )->LeftX() <= rs_seed_rx ) {
			if ( rs_seed_source_value == rs->SourceValue()) {
				rspi = rspl_p->erase( rspi );
				seed_queue.push( rs );
			} else {
				rspi++;
			}
			if ( rspi == rspl_p->end()) {
				return;
			}
		}

		return;
#endif
#if 0
		while ( rspi != rspl_p->end()) {
			RasterSegment	*rs = *rspi;
			if ( rs_seed_source_value == rs->SourceValue() 
				 && rs_seed_lx <= rs->RightX()
				 && rs_seed_rx >= rs->LeftX()) {
				rspi = rspl_p->erase( rspi );
				seed_queue.push( rs );
			} else {
				rspi++;
			}
		}
#endif	
	}

	RegionInfo *
	ConnectRasterSegment( RasterSegment *rs_seed,
						  const DstT region_num )
	{
		RegionInfo	*ri = new RegionInfo;

		int	num_of_pixels = 0;
		int	min_x, min_y;
		int	max_x, max_y;
		SrcT	source_value;

		min_x = rs_seed->LeftX();
		max_x = rs_seed->RightX();
		min_y = max_y = rs_seed->Y();
		source_value = rs_seed->SourceValue();

#if CALC_CENTER_OF_GRAVITY
		int	sum_x = 0;
		int	sum_y = 0;
#endif
	
		seed_queue.push( rs_seed );

		while ( seed_queue.size() > 0 ) {
			RasterSegment	*rs = seed_queue.front();
			seed_queue.pop();
			ri->Push( rs );

			int	n = rs->RightX() - rs->LeftX() + 1;
			num_of_pixels += n;
			if ( rs->LeftX() < min_x ) {
				min_x = rs->LeftX();
			}
			if ( rs->RightX() > max_x ) {
				max_x = rs->RightX();
			}
			if ( rs->Y() < min_y ) {
				min_y = rs->Y();
			} else if ( rs->Y() > max_y ) {
				max_y = rs->Y();
			}
#if CALC_CENTER_OF_GRAVITY
			sum_x += ( rs->LeftX() + rs->RightX()) * n;
			sum_y += rs->Y() * n;
#endif			

			if ( rs->Y() > 0 ) {
				SearchNeighboringSegment( rs, -1 );
			}
			if ( rs->Y() < height - 1 ) {
				SearchNeighboringSegment( rs, 1 );
			}
		}

		ri->SetNumOfPixels( num_of_pixels );
		ri->SetMinMax( min_x, min_y, max_x, max_y );
		ri->SetSourceValue( source_value );
		ri->SetResult( region_num );
#if CALC_CENTER_OF_GRAVITY
		float	gx = static_cast<float>( sum_x ) / ( 2 * num_of_pixels );
		float	gy = static_cast<float>( sum_y ) / num_of_pixels;
		ri->SetCenterOfGravity( gx, gy );
#endif
		return ri;
	}

	static	bool
	RevCompRegionInfoPointer( const RegionInfo * const &l,
							  const RegionInfo * const &r )
	{
		bool	b = ( l->GetNumOfPixels() > r->GetNumOfPixels());
		if ( l->GetNumOfPixels() == r->GetNumOfPixels()) {
			int	lx, ly, rx, ry;
			l->GetMin( lx, ly );
			r->GetMin( rx, ry );
			b = ( ly > ry );
		}
		return b;
	}

	void
	FillFrame( RegionInfo *ri, const DstT fill_value )
	{
#if 0
		while ( ri->GetNumOfRasterSegments() > 0 ) {
			RasterSegment	*rs;
			ri->Pop( rs );
			DstT	*sp = dst_frame + rs->LeftX() + rs->Y() * width;
			for ( int i = 0; i < rs->RightX() - rs->LeftX() + 1; i++ ) {
				*sp++ = fill_value;
			}
		}
#endif
		RSPList	rspl = ri->GetRasterSegmentList();
		for ( RSPIterator rspi = rspl.begin(); rspi != rspl.end(); rspi++ ) {
			RasterSegment	*rs = *rspi;
			int	lx = rs->LeftX();
			int	rx = rs->RightX();
			int	y = rs->Y();
			DstT	*sp = dst_frame + lx + y * width;
			for ( int i = 0; i < ( rx - lx + 1 ); i++ ) {
				*sp++ = fill_value;
			}
		}
	}

public:

	inline	int
	GetNumOfRegions( void )	const
	{
		return num_of_regions;
	}
	
	inline	int
	GetNumOfResultRegions( void )	const
	{
		return num_of_result_regions;
	}

	inline	RegionInfo *
	GetResultRegionInfo( const int num )	const
	{
		return result_region_info[ num ];
	}

	Labeling()
	{
		raster_segment_list = 0;
		region_info_list.clear();
		result_region_info.clear();
	}

	virtual ~Labeling()
	{
		for ( RIPIterator ripi = region_info_list.begin();
			  ripi != region_info_list.end(); ripi++ ) {
			RegionInfo	*ri = *ripi;
			delete ri;
		}
		region_info_list.erase( region_info_list.begin(),
								region_info_list.end());
		result_region_info.clear();
	}

#define	CHECK_FOR_PHASE1	0
#define	CHECK_FOR_PHASE2	0

	int
	Exec( SrcT *target, DstT *result,
		  int target_width, int target_height,
		  const bool is_sort_region,
		  const int region_size_min )
	{
		src_frame = target;
		dst_frame = result;

		width     = target_width;
		height    = target_height;
		total_num = width * height;

		// phase pre1

		for ( RIPIterator ripi = region_info_list.begin();
			  ripi != region_info_list.end(); ripi++ ) {
			RegionInfo	*ri = *ripi;
			delete ri;
		}
		region_info_list.erase( region_info_list.begin(),
								region_info_list.end());
		result_region_info.clear();

		raster_segment_list = new RSPList[ height ];
		num_of_raster_segments = 0;

		// phase 1

		SrcT	*p = src_frame;

#if ( CLEAR_DST_BUFFER || CLEAR_ALL_DST_BUFFER )
		DstT	*q = dst_frame;
#endif
		if ( src_frame != reinterpret_cast<SrcT *>( dst_frame )) {
#if CLEAR_ALL_DST_BUFFER
			for ( int i = 0; i < width * height; i++ ) {
				*q++ = 0;
			}
#endif		
			for ( int y = 0; y < height; y++ ) {
				int	lx = 0;
				int	current_src_value = 0;
				for ( int x = 0; x < width; x++ ) {
					if ( *p != current_src_value ) {
						if ( current_src_value != 0 ) {	// raster segment
							RegisterSegment( lx, x - 1, y, current_src_value );
						}
						current_src_value = *p;
						lx = x;
					}
#if ( CLEAR_DST_BUFFER && !CLEAR_ALL_DST_BUFFER )
					if ( *p == 0 ) {	// if src = 0
						*q = 0;			// clear destination buffer
					}
					q++;
#endif				
					p++;
				}
				if ( current_src_value != 0 ) {
					RegisterSegment( lx, width - 1, y, current_src_value );
				}
			}
		} else {	// no need to clear dst_frame if src_frame = dst_frame
			for ( int y = 0; y < height; y++ ) {
				int	lx = 0;
				int	current_src_value = 0;
				for ( int x = 0; x < width; x++ ) {
					if ( *p != current_src_value ) {
						if ( current_src_value != 0 ) {	// raster segment
							RegisterSegment( lx, x - 1, y, current_src_value );
						}
						current_src_value = *p;
						lx = x;
					}
					p++;
				}
				if ( current_src_value != 0 ) {
					RegisterSegment( lx, width - 1, y, current_src_value );
				}
			}
		}

#if	CHECK_FOR_PHASE1
		for ( int y = 0; y < height; y++ ) {
			cout << y << ":" << raster_segment_list[ y ].size() << endl;
			RSPList	*rspl_p = &raster_segment_list[ y ];
			RSPIterator	i;
			for ( i = rspl_p->begin(); i != rspl_p->end(); i++ ) {
				RasterSegment	*rs = *i;
				cout << *rs;
			}
		}
		cout << "num_of_raster_segments: " << num_of_raster_segments << endl;
#endif

		// phase pre2

		region_info_list.clear();
		num_of_regions = 0;

		// phase 2: connect

		for ( int y = 0; y < height; y++ ) {
			RSPList	*rspl_p = &raster_segment_list[ y ];
			while ( rspl_p->size() > 0 ) {
				RSPIterator	rspi = rspl_p->begin();
				RasterSegment	*rs = *rspi;	// get 1 raster segment
				rspl_p->erase( rspi );			// remove from list

				RegionInfo	*rip = ConnectRasterSegment( rs,
														 num_of_regions + 1 );
				region_info_list.push_back( rip );
				num_of_regions++;
			}
		}

#if	CHECK_FOR_PHASE2
		for ( int y = 0; y < height; y++ ) {
			if ( !raster_segment_list[ y ].empty()) {
				cout << "mmmm" << y << endl;
			}
		}

		int	n_p = 0;
		for ( RIPIterator ripi = region_info_list.begin();
			  ripi != region_info_list.end(); ripi++ ) {
			RegionInfo	*ri = *ripi;
			n_p += ri->GetNumOfPixels();
			while ( ri->GetNumOfRasterSegments() > 0 ) {
				RasterSegment	*rs;
				ri->Pop( rs );
				cout << *rs;
			}
		}
		cout << "num_of_pixels: " << n_p << endl;
		cout << "num_of_regions: " << num_of_regions << endl;
#endif

		// phase 3
		// reorder by size

		result_region_info.resize( num_of_regions );
		int	n = 0;
		for ( RIPIterator ripi = region_info_list.begin();
			  ripi != region_info_list.end(); ripi++ ) {
			result_region_info[ n ] = *ripi;
			n++;
		}

		if ( is_sort_region ) {
			// sort result_region_info by size

			sort( result_region_info.begin(), result_region_info.end(),
				  RevCompRegionInfoPointer );
		}

		// renumber IDs of RegionInfo

		if ( is_sort_region && region_size_min > 0 ) {
			int	n = 0;
			while ( n < num_of_regions
					&& result_region_info[ n ]->GetNumOfPixels()
					>= region_size_min ) {
				result_region_info[ n ]->SetResult( n + 1 );
				n++;
			}
			num_of_result_regions = n;
			for ( int i = n; i < num_of_regions; i++ ) {
				result_region_info[ i ]->SetResult( 0 );
			}
		} else {
			for ( int i = 0; i < num_of_regions; i++ ) {
				result_region_info[ i ]->SetResult( i + 1 );
			}
			num_of_result_regions = num_of_regions;
		}

		// phase 4
		// put label number for pixels

		for ( int i = 0; i < num_of_regions; i++ ) {
			RegionInfo	*ri = result_region_info[ i ];
			FillFrame( ri, ri->GetResult());
		}

		// clear

		delete [] raster_segment_list;

		return 0;
	}
};

typedef Labeling<unsigned char,short> LabelingBS;
typedef Labeling<short,short> LabelingSS;
typedef Labeling<unsigned char,short>::RegionInfo RegionInfoBS;
typedef Labeling<short,short>::RegionInfo RegionInfoSS;

#endif	// __LABELING_H__
