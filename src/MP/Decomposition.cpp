#include "MP/Decomposition.hpp"
#include "MP/Planner.hpp"
#include "MP/Params.hpp"
#include "External/ShewchukTriangle.hpp"
#include "Utils/GDraw.hpp"

namespace Abetare
{
    void Decomposition::CompleteSetup(void)
    {
	Problem *prob = &(Planner::GetInstance()->m_problem);
	
	m_covGrid.Setup2D(PARAM_DECOMPOSITION_COV_GRID_DIMX,
			  PARAM_DECOMPOSITION_COV_GRID_DIMY,
			  prob->m_grid.GetMin()[0], 
			  prob->m_grid.GetMin()[1], 
			  prob->m_grid.GetMax()[0], 
			  prob->m_grid.GetMax()[1]);
	Triangulate();
	LocatorConstruct();
    }
    
    int Decomposition::LocateRegion(const double p[])
    {
	const int cid = m_covGrid.GetCellId(p);
	
	if(cid < 0 || cid >= m_covGrid.GetNrCells())
	    return -1;
	
	if(m_cellsToRegions[cid]->m_insideRegion >= 0)
	    return m_cellsToRegions[cid]->m_insideRegion;
	
	for(int i = 0; i < m_cellsToRegions[cid]->m_intersectRegions.size(); ++i)
	{
	    const std::vector<double> *poly = 
		&(m_regions[m_cellsToRegions[cid]->m_intersectRegions[i]]->m_shape.m_vertices);
	    
	    if(IsPointInsidePolygon2D(p, poly->size() / 2, &((*poly)[0])))
		return m_cellsToRegions[cid]->m_intersectRegions[i];
	}	
	return -1;
    }


    void Decomposition::LocatorConstruct(void)
    {
//construct cells to region map
	const int nrGridCells = m_covGrid.GetNrCells();
	m_cellsToRegions.resize(nrGridCells);
	for(int i = 0; i < nrGridCells; ++i)
	{
	    m_cellsToRegions[i] = new InsideIntersectRegions();
	    m_cellsToRegions[i]->m_insideRegion = -1;
	}
	
	const int nr = m_regions.size();
	for(int i = 0; i < nr; ++i)
	    LocatorUpdateCells(i);
    }

    
    void Decomposition::LocatorUpdateCells(const int rid)
    {
	Region *r = m_regions[rid];
	
	r->m_shape.OccupiedGridCells(&m_covGrid, &(r->m_cellsInside), &(r->m_cellsIntersect));
	for(int j = 0; j < r->m_cellsInside.size(); ++j)
	    m_cellsToRegions[r->m_cellsInside[j]]->m_insideRegion = rid;
	for(int j = 0; j < r->m_cellsIntersect.size(); ++j)
	    m_cellsToRegions[r->m_cellsIntersect[j]]->m_intersectRegions.push_back(rid);
    }

    void Decomposition::Draw(void)
    {
	DrawRegions();
//	DrawEdges();
    }
    
    void Decomposition::DrawRegions(void)
    {
	const bool is2D = GDrawIs2D();
	
	GDraw2D();
	GDrawWireframe(true);
	GDrawLineWidth(1.0);
	GDrawColor(0.6, 0.6, 0.2); 
	GDrawPushTransformation();
	GDrawMultTrans(0, 0, 0.2);
	 
	if(!is2D)
	    GDrawSetParam(GDRAW2D, 0);
	for(int i = m_regions.size() - 1; i >= 0; --i)
	    m_regions[i]->m_shape.Draw();
	GDrawLineWidth(1.0);
	GDrawWireframe(false);
	GDrawPopTransformation();
	
	if(!is2D)
	    GDraw3D();
    }


    void Decomposition::DrawEdges(void)
    {
	double     p1[3];
	double     p2[3];
	const bool is2D = GDrawIs2D();
	
	GDraw2D();
	GDrawWireframe(true);
	GDrawLineWidth(2.0);
	GDrawColor(1.0, 1.0, 0.0);
	
	for(int i = m_regions.size() - 1; i >= 0; --i)
	{
	    m_regions[i]->m_shape.GetSomePointInside(p1);
	    if(m_regions[i]->m_valid)
		for(auto neigh = m_regions[i]->m_neighs.begin(); neigh != m_regions[i]->m_neighs.end(); ++neigh)
		    if(i > (*neigh))
		    {
			m_regions[*neigh]->m_shape.GetSomePointInside(p2);
			GDrawSegment3D(p1[0], p1[1], is2D ? 0 : 0.2,
				       p2[0], p2[1], is2D ? 0 : 0.2);
		    }
	    
	}
	
	GDrawLineWidth(1.0);
	GDrawWireframe(false);
	if(!is2D)
	    GDraw3D();
    }
    
    int Decomposition::GetDepotId(const int i) const
    {
	return i;
    }
    
    int Decomposition::GetDoorId(const int i) const
    {
	return i + Planner::GetInstance()->m_problem.m_rooms.size();
    }
    
    
    void Decomposition::Triangulate(void)
    {	
	Problem            *prob    = &(Planner::GetInstance()->m_problem);
	const int           nrObsts = prob->m_obstacles.m_polys.size();
	const int           nrRooms = prob->m_rooms.size();
	const int           nrDoors = prob->m_doors.size();
	const int           nrProps = nrRooms + nrDoors;
	const double       *pmin    = m_covGrid.GetMin();
	const double       *pmax    = m_covGrid.GetMax();
	Polygon2D           depot;
	std::vector<double> vertices;
	std::vector<int>    nrVerticesPerContour;
	std::vector<double> ptsInsideHoles;
	int                 nrInvalid = 0;
	

	
//grid boundaries	
	vertices.push_back(pmin[0]); vertices.push_back(pmin[1]);
	vertices.push_back(pmax[0]); vertices.push_back(pmin[1]);
	vertices.push_back(pmax[0]); vertices.push_back(pmax[1]);
	vertices.push_back(pmin[0]); vertices.push_back(pmax[1]);
	nrVerticesPerContour.push_back(4);	   
 

//obstacles and goals as holes
	ptsInsideHoles.resize(2 * (nrObsts + nrProps));	  

	//room depots
	depot.m_vertices.resize(8);
	for(int i = 0; i < nrRooms; ++i)
	{
	    AABoxAsPolygon2D(prob->m_rooms[i]->m_depot, &(prob->m_rooms[i]->m_depot[2]), &(depot.m_vertices[0]));
	    depot.OnShapeChange();
	    nrVerticesPerContour.push_back(depot.m_vertices.size() / 2);
	    depot.GetSomePointInside(&ptsInsideHoles[2 * i]);
	    vertices.insert(vertices.end(), 
			    depot.m_vertices.begin(), 
			    depot.m_vertices.end());
	}
	//doors
	depot.m_vertices.resize(8);
	for(int i = 0; i < nrDoors; ++i)
	{
	    AABoxAsPolygon2D(prob->m_doors[i]->m_box, &(prob->m_doors[i]->m_box[2]), &(depot.m_vertices[0]));
	    depot.OnShapeChange();
	    nrVerticesPerContour.push_back(depot.m_vertices.size() / 2);
	    depot.GetSomePointInside(&ptsInsideHoles[2 * nrRooms + 2 * i]);
	    vertices.insert(vertices.end(), 
			    depot.m_vertices.begin(), 
			    depot.m_vertices.end());
	}


	
	//obstacles
	for(int i = 0; i < nrObsts; ++i)
	{
	    nrVerticesPerContour.push_back(prob->m_obstacles.m_polys[i]->m_vertices.size() / 2);
	    prob->m_obstacles.m_polys[i]->GetSomePointInside(&ptsInsideHoles[2 * nrProps + 2 * i]);
	    vertices.insert(vertices.end(), 
			    prob->m_obstacles.m_polys[i]->m_vertices.begin(), 
			    prob->m_obstacles.m_polys[i]->m_vertices.end());
	}
	
	std::vector<double> triVertices;
	std::vector<int>    triIndices;
	std::vector<int>    triNeighs;
	
	TriangulatePolygonWithHoles2D(false, 20, 10.0,
				      vertices.size() / 2, 
				      &vertices[0],
				      &nrVerticesPerContour[0],
				      ptsInsideHoles.size() / 2, &ptsInsideHoles[0],
				      &triVertices, &triIndices, &triNeighs);

//construct decomposition
	Region      *region  = NULL;	
	const int    nrTris3 = triIndices.size();
	double       pmin1[2];
	double       pmin2[2];
	
	//add rooms
	for(int i = 0; i < nrRooms; ++i)
	{
	    region          = new Region();
	    region->m_type  = Region::TYPE_DEPOT;
	    region->m_label = i;
	    region->m_shape.m_vertices.resize(8);
	    AABoxAsPolygon2D(prob->m_rooms[i]->m_depot, &(prob->m_rooms[i]->m_depot[2]), &(region->m_shape.m_vertices[0]));
	    region->m_shape.OnShapeChange();
	    region->m_shape.GetSomePointInside(region->m_center);
	    m_regions.push_back(region);
	}

	//add doors
	for(int i = 0; i < nrDoors; ++i)
	{
	    region          = new Region();
	    region->m_type  = Region::TYPE_DOOR;
	    region->m_label = i;
	    region->m_shape.m_vertices.resize(8);
	    AABoxAsPolygon2D(prob->m_doors[i]->m_box, &(prob->m_doors[i]->m_box[2]), &(region->m_shape.m_vertices[0]));
	    region->m_shape.OnShapeChange();
	    region->m_shape.GetSomePointInside(region->m_center);
	    m_regions.push_back(region);
	}


	//add triangle regions
	double p[2];
	
	for(int i = 0; i < nrTris3; i += 3)
	{
	    region = new Region();
	    region->m_type  = Region::TYPE_NONE;
	    region->m_shape.m_vertices.push_back(triVertices[    2 * triIndices[i]]);
	    region->m_shape.m_vertices.push_back(triVertices[1 + 2 * triIndices[i]]);
	    region->m_shape.m_vertices.push_back(triVertices[    2 * triIndices[i + 1]]);
	    region->m_shape.m_vertices.push_back(triVertices[1 + 2 * triIndices[i + 1]]);
	    region->m_shape.m_vertices.push_back(triVertices[    2 * triIndices[i + 2]]);
	    region->m_shape.m_vertices.push_back(triVertices[1 + 2 * triIndices[i + 2]]);
	    MakePolygonCCW2D(3, &region->m_shape.m_vertices[0]);
	    region->m_valid = region->m_shape.GetArea() > PARAM_DECOMPOSITION_MIN_AREA_TO_ADD;

	    region->m_shape.GetSomePointInside(region->m_center);
	    region->m_label = prob->m_grid.GetCellId(region->m_center);
	    
	    m_regions.push_back(region);

	    if(region->m_valid == false)
		++nrInvalid;
	    
	}

	
//no edge if a triangle's area is small or if it happens to be in collision
	for(int i = 0; i < nrTris3; i += 3)
	{
	    region = m_regions[nrProps + i/3];
	 
	    if(region->m_valid == false)
		continue;
	    
	    if(triNeighs[i] >= 0 && m_regions[nrProps + triNeighs[i]]->m_valid == true)
		region->m_neighs.push_back(nrProps + triNeighs[i]);
	    if(triNeighs[i + 1] >= 0  && m_regions[nrProps + triNeighs[i + 1]]->m_valid == true)
		region->m_neighs.push_back(nrProps + triNeighs[i + 1]);
	    if(triNeighs[i + 2] >= 0  && m_regions[nrProps + triNeighs[i + 2]]->m_valid == true) 
		region->m_neighs.push_back(nrProps + triNeighs[i + 2]);
	    
	    if(triNeighs[i] < 0 || triNeighs[i + 1] < 0 || triNeighs[i + 2] < 0)
	    {
		for(int j = 0; j < nrProps; ++j)
		    if(DistSquaredPolygons2D(region->m_shape.m_vertices.size() / 2,
					     &(region->m_shape.m_vertices[0]),
					     m_regions[j]->m_shape.m_vertices.size() / 2,
					     &m_regions[j]->m_shape.m_vertices[0], pmin1, pmin2)
		       <= Constants::EPSILON_SQUARED)
		    {
			region->m_neighs.push_back(j);
			m_regions[j]->m_neighs.push_back(nrProps + i / 3);
		    }
	    }
	}


//construct weights
	int nedges = 0;
	for(int i = 0; i < m_regions.size(); ++i)
	{
	    const int k = m_regions[i]->m_neighs.size();
	    m_regions[i]->m_weights.resize(k);	    
	    for(int j = 0; j < k; ++j)
	 	m_regions[i]->m_weights[j] = Algebra2D::PointDist(m_regions[i]->m_center, m_regions[m_regions[i]->m_neighs[j]]->m_center);
	    nedges += k;
	}

	printf("...............abstraction has %d vertices  (%d invalid) and %d edges\n", (int) m_regions.size(), nrInvalid, nedges);
	
	
    }
    
}

