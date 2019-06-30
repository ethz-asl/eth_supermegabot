# Some basic tools for polygonal geometry

# Convex hull part sourced from:
#     https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#Python
# Partially adapted from C++ code Dan Sunday, at http://geomalgorithms.com/a09-_intersect-3.html
# // Copyright 2001 softSurfer, 2012 Dan Sunday
# // This code may be freely used and modified for any purpose
# // providing that this copyright notice is included with it.
# // SoftSurfer makes no warranty for this code, and cannot be held
# // liable for any real or imagined damage resulting from its use.
# // Users of this code must verify correctness for their application.


# // Assume that classes are already given for the objects:
# //    Point with 2D coordinates {float x, y;}
# //    Polygon with n vertices {int n; Point *V;} with V[n]=V[0]
# //    Tnode is a node element structure for a BBT
# //    BBT is a class for a Balanced Binary Tree
# //        such as an AVL, a 2-3, or a  red-black tree
# //        with methods given by the  placeholder code:

from collections import namedtuple

# Points should be 2-tuple (x,y)
Point = namedtuple('Point', ('x', 'y'))
OrderedEdge = namedtuple('OrderedEdge', ('lp', 'rp'))


class PointList(list):
    bounds = None

    def min_yx_index(self):
        im = 0
        for i, pi in enumerate(self):
            if pi.y < self[im].y:
                im = i
            elif pi.y == self[im].y and pi.x < self[im].x:
                im = i
        return im

    def swap(self, i, j):
        self[i], self[j] = self[j], self[i]

    def get_bounds(self):
        # returns [minx, maxx, miny, maxy]
        if self.bounds is None:
            self.bounds = [min(self, key=lambda t:t[0])[0],
                           max(self, key=lambda t:t[0])[0],
                           min(self, key=lambda t:t[1])[1],
                           max(self, key=lambda t:t[1])[1]]
        return self.bounds

    def get_xy(self):
        x, y = zip(*self)


class Polygon(PointList):
    # List of points (with closure), use method edges() to get iterator over edges
    # This automatically closes the polygon, so that Polygon[n] = Polygon[0], but length (n) is still number of verts

    # def __getitem__(self, key):
    #     if key == len(self):
    #         return super(Polygon, self).__getitem__(0)
    #     else:
    #         return super(Polygon, self).__getitem__(key)
    #
    # def __iter__(self):
    #     for p in super(Polygon, self).__iter__():
    #         yield p
    #     yield self[0]
    #
    # def vertices(self):
    #     return self[:len(self)]

    def edges(self):
        for i in range(len(self)-1):
            yield self[i], self[i+1]
        yield self[-1], self[0]

    def get_edge(self, i):
        return self[i], self[(i+1) % len(self)]

    def point_inside_cn(self, p):
        #  crossing_number_poly(): crossing number test for a point in a polygon
        #       Input:   P = a point,
        #                V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
        #       Return:  0 = outside, 1 = inside
        #  This code is patterned after [Franklin, 2000] (normally just use winding method)
        cn = 0    # crossing number counter

        #  loop through all edges of the polygon
        for v0, v1 in self.edges():    # edge from V[i]  to V[i+1]
            if ((v0.y <= p.y) and (v1.y > p.y)) or ((v0.y > p.y) and (v1.y <= p.y)):    # a downward crossing
                # Compute  the actual edge-ray intersect x-coordinate
                vt = (p.y - v0.y) / (v1.y - v0.y)
                if p.x <  v0.x + vt * (v1.x - v0.x):  # P.x < intersect
                     cn += 1   # a valid crossing of y=P.y right of P.x

        return bool(cn % 2)    # 0 if even (out), and 1 if  odd (in)

    def point_inside(self, p):
        #  point_inside(): winding number test for a point in a polygon
        #       Input:   p = a point,
        #       Return:  wn = the winding number (=0 only when P is outside)

        wn = 0  # the  winding number counter

        #  loop through all edges of the polygon
        for v0, v1 in self.edges():  # edge from V[i] to  V[i+1]
            if v0.y <= p.y:  # start y <= P.y
                if v1.y > p.y:  # an upward crossing
                    if is_left(v0, v1, p) > 0:  # P left of  edge
                        wn += 1  # have  a valid up intersect

            else:  # start y > P.y (no test needed)
                if v1.y <= p.y:  # a downward crossing
                    if is_left(v0, v1, p) < 0:  # P right of  edge
                        wn -= 1  # have  a valid down intersect
        return wn

    def intersect(self, poly2):
        assert isinstance(poly2, Polygon)
        bounds1 = self.get_bounds()
        bounds2 = poly2.get_bounds()

        if (bounds2[1] <= bounds1[0] or bounds2[0] >= bounds1[1] or
                bounds2[3] <= bounds1[2] or bounds2[2] >= bounds1[3]):
            return False

        for p in poly2:
            if self.point_inside(p):
                return True
        for p in self:
            if poly2.point_inside(p):
                return True

        all_edges = []
        def add_ordered_edges(edge_list, new_edges):
            for p0, p1 in new_edges:
                if p0 < p1:
                    edge_list.append(OrderedEdge(p0, p1))
                else:
                    edge_list.append(OrderedEdge(p1, p0))

        my_edges = []
        add_ordered_edges(my_edges, self.edges())
        your_edges = []
        add_ordered_edges(your_edges, poly2.edges())
        for e1 in my_edges:
            for e2 in your_edges:
                if line_intersect(e1, e2):
                    return True
        return False

        """ Something (occasionally) wrong with my edge ordering in Shamos-Hoey booooooo~~ """
        # add_ordered_edges(all_edges, self.edges())
        # add_ordered_edges(all_edges, poly2.edges())
        #
        # event_queue = EventQueue(all_edges)
        # segment_list = []
        # def find_list_index(seglist, new_edge):
        #     i = -1
        #     for i, edge_index in enumerate(seglist):
        #         if is_left(new_edge.vertex, *all_edges[edge_index]):
        #             return i
        #     return i+1
        #
        # for e in event_queue.events:
        #
        #     if e.is_left_end:
        #         # We're adding it to the segment list
        #         # If it's a left end (new segment), check who is above, put in the list
        #         i = find_list_index(segment_list, e)
        #         segment_list.insert(i, e.edge_id)
        #         if i > 0 and line_intersect(all_edges[e.edge_id], all_edges[segment_list[i-1]]):
        #             return True
        #         if i < (len(segment_list)-1) and line_intersect(all_edges[e.edge_id], all_edges[segment_list[i+1]]):
        #             return True
        #
        #     else:
        #         # It's a right end and we need to pull it out of the queue
        #         i = segment_list.index(e.edge_id)
        #         if 0 < i < (len(segment_list)-1) and line_intersect(all_edges[segment_list[i-1]], all_edges[segment_list[i+1]]):
        #             return True
        #         segment_list.pop(i)
        #
        # return False

def line_intersect(l0, l1):
    # Assume ordered lines (OrderedEdge objects)
    lsign = is_left(l0.lp, l0.rp, l1.lp)    #  l1 left point sign
    rsign = is_left(l0.lp, l0.rp, l1.rp)    #  l1 right point sign
    if (lsign * rsign >= 0):                 # l1 endpoints have same sign  relative to l0
        return False                        # => on same side => no intersect is possible
    lsign = is_left(l1.lp, l1.rp, l0.lp)    #  l0 left point sign
    rsign = is_left(l1.lp, l1.rp, l0.rp)    #  l0 right point sign
    if (lsign * rsign >= 0):                 # l0 endpoints have same sign  relative to l1
        return False                        # => on same side => no intersect is possible
    return True                             # => an intersect exists


def convex_hull(points, return_copy=False):
    """Computes the convex hull of a set of 2D points.

        Input: an iterable sequence of (x, y) pairs representing the points.
        Output: a list of vertices of the convex hull in counter-clockwise order,
          starting from the vertex with the lexicographically smallest coordinates.
        Implements Andrew's monotone chain algorithm. O(n log n) complexity.
    """
    if return_copy:
        raise NotImplementedError

    # Sort the points lexicographically (tuples are compared lexicographically).
    # Remove duplicates to detect the case we have just one unique point.
    points = sorted(set(points))

    # Boring case: no points or a single point, possibly repeated multiple times.
    if len(points) <= 1:
        return points

    def ccw(p1, p2, p3):
        # Three points are a counter-clockwise turn if ccw > 0, clockwise if
        # ccw < 0, and collinear if ccw = 0 because ccw is a determinant that
        # gives twice the signed  area of the triangle formed by p1, p2 and p3
        return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)

    # Build lower hull
    lower = PointList([])
    for p in points:
        while len(lower) >= 2 and ccw(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Build upper hull
    upper = PointList([])
    for p in reversed(points):
        while len(upper) >= 2 and ccw(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenation of the lower and upper hulls gives the convex hull.
    # Last point of each list is omitted because it is repeated at the beginning of the other list.
    return Polygon(lower[:-1] + upper[:-1])


def xy_order(p1, p2):
    # Determines the xy lexicographical order of two points
    # Returns: (+1) if p1 > p2; (-1) if p1 < p2; and  0 if equal
    if p1.x > p2.x:
        return 1
    if p1.x < p2.x:
        return -1
    # tiebreak with y
    if p1.y > p2.y:
        return 1
    if p1.y < p2.y:
        return -1
    # otherwise same point
    return 0


def is_left(p0, p1, p2):
    # tests if point P2 is Left|On|Right of the line P0 to P1.
    # returns: >0 for left, 0 for on, and <0 for  right of the line.
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)

# ===================================================================


class Event:
    def __init__(self, edge_id, is_left_end, vertex):
        self.edge_id = edge_id
        self.is_left_end = is_left_end
        self.vertex = vertex


def event_compare(event0, event1):
    # Compare (order) two events
    return xy_order(event0.vertex, event1.vertex)


class EventQueue(object):

    def __init__(self, edge_list):
        self.events =[]
        for i, (v0, v1) in enumerate(edge_list):
            left_first = (v0 <= v1)  # (xy_order(v0, v1) >= 0)
            self.events.append(Event(i, left_first, v0))
            self.events.append(Event(i, (not left_first), v1))

        self.events = sorted(self.events, cmp=event_compare)

    def pop(self, index=-1):
        return self.events.pop(index)



class SweepLineSegment(object):

    def __init__(self, edge_id, p_left, p_right, above=None, below=None):
        self.edge_id = edge_id
        self.p_left = p_left
        self.p_right = p_right
        self.above = above
        self.below = below

    def set_above(self, above):
        self.above = above

    def set_below(self, below):
        self.below = below

class SweepLine:

    def __init__(self, poly):
        self.nv = len(poly)
        self.poly = poly
        self.tree = {}


    def add(self, event):
        v0, v1 = self.poly.get_edge(event.edge_id)
        if v0 < v1:
            new_seg = SweepLineSegment(event.edge_id, p_left=v0, p_right=v1)
        else:
            new_seg = SweepLineSegment(event.edge_id, p_left=v1, p_right=v0)
        self.tree[event.edge_id] = new_seg



#
# // SweepLine Class
#
# // SweepLine segment data struct
# typedef struct _SL_segment SLseg;
# struct _SL_segment {
#     int      edge;          // polygon edge i is V[i] to V[i+1]
#     Point    lp;            // leftmost vertex point
#     Point    rp;            // rightmost vertex point
#     SLseg*   above;         // segment above this one
#     SLseg*   below;         // segment below this one
# };
#
# // the Sweep Line itself
# class SweepLine {
#     int      nv;            // number of vertices in polygon
#     Polygon* Pn;            // initial Polygon
#     BBT      Tree;          // balanced binary tree
# public:
#               SweepLine(Polygon P)            // constructor
#                   { nv = P.n; Pn = &P; }
#              ~SweepLine(void)                 // destructor
#                   { Tree.freetree();}
#
#     SLseg*   add( Event* );
#     SLseg*   find( Event* );
#     int      intersect( SLseg*, SLseg*  );
#     void     remove( SLseg* );
# };
#
# SLseg* SweepLine::add( Event* E )
# {
#     // fill in SLseg element data
#     SLseg* s = new SLseg;
#     s->edge  = E->edge;
#
#     // if it is being added, then it must be a LEFT edge event
#     // but need to determine which endpoint is the left one
#     Point* v1 = &(Pn->V[s->edge]);
#     Point* v2 = &(Pn->V[s->edge+1]);
#     if (xyorder( v1, v2) < 0) { // determine which is leftmost
#         s->lp = *v1;
#         s->rp = *v2;
#     }
#     else {
#         s->rp = *v1;
#         s->lp = *v2;
#     }
#     s->above = (SLseg*)0;
#     s->below = (SLseg*)0;
#
#     // add a node to the balanced binary tree
#     Tnode* nd = Tree.insert(s);
#     Tnode* nx = Tree.next(nd);
#     Tnode* np = Tree.prev(nd);
#     if (nx != (Tnode*)0) {
#         s->above = (SLseg*)nx->val;
#         s->above->below = s;
#     }
#     if (np != (Tnode*)0) {
#         s->below = (SLseg*)np->val;
#         s->below->above = s;
#     }
#     return s;
# }
#
# SLseg* SweepLine::find( Event* E )
# {
#     // need a segment to find it in the tree
#     SLseg* s = new SLseg;
#     s->edge  = E->edge;
#     s->above = (SLseg*)0;
#     s->below = (SLseg*)0;
#
#     Tnode* nd = Tree.find(s);
#     delete s;
#     if (nd == (Tnode*)0)
#         return (SLseg*)0;
#
#     return (SLseg*)nd->val;
# }
#
# void SweepLine::remove( SLseg* s )
# {
#     // remove the node from the balanced binary tree
#     Tnode* nd = Tree.find(s);
#     if (nd == (Tnode*)0)
#         return;       // not there
#
#     // get the above and below segments pointing to each other
#     Tnode* nx = Tree.next(nd);
#     if (nx != (Tnode*)0) {
#         SLseg* sx = (SLseg*)(nx->val);
#         sx->below = s->below;
#     }
#     Tnode* np = Tree.prev(nd);
#     if (np != (Tnode*)0) {
#         SLseg* sp = (SLseg*)(np->val);
#         sp->above = s->above;
#     }
#     Tree.remove(nd);       // now  can safely remove it
#     delete s;
# }
#
# // test intersect of 2 segments and return: 0=none, 1=intersect
# int SweepLine::intersect( SLseg* s1, SLseg* s2)
# {
#     if (s1 == (SLseg*)0 || s2 == (SLseg*)0)
#         return FALSE;       // no intersect if either segment doesn't exist
#
#     // check for consecutive edges in polygon
#     int e1 = s1->edge;
#     int e2 = s2->edge;
#     if (((e1+1)%nv == e2) || (e1 == (e2+1)%nv))
#         return FALSE;       // no non-simple intersect since consecutive
#
#     // test for existence of an intersect point
#     float lsign, rsign;
#     lsign = isLeft(s1->lp, s1->rp, s2->lp);    //  s2 left point sign
#     rsign = isLeft(s1->lp, s1->rp, s2->rp);    //  s2 right point sign
#     if (lsign * rsign > 0) // s2 endpoints have same sign  relative to s1
#         return FALSE;       // => on same side => no intersect is possible
#     lsign = isLeft(s2->lp, s2->rp, s1->lp);    //  s1 left point sign
#     rsign = isLeft(s2->lp, s2->rp, s1->rp);    //  s1 right point sign
#     if (lsign * rsign > 0) // s1 endpoints have same sign  relative to s2
#         return FALSE;       // => on same side => no intersect is possible
#     // the segments s1 and s2 straddle each other
#     return TRUE;            // => an intersect exists
# }
# //===================================================================
#
#
#
# // simple_Polygon(): test if a Polygon is simple or not
# //     Input:  Pn = a polygon with n vertices V[]
# //     Return: FALSE(0) = is NOT simple
# //             TRUE(1)  = IS simple
# int
# simple_Polygon( Polygon Pn )
# {
#     EventQueue  Eq(Pn);
#     SweepLine   SL(Pn);
#     Event*      e;                  // the current event
#     SLseg*      s;                  // the current SL segment
#
#     // This loop processes all events in the sorted queue
#     // Events are only left or right vertices since
#     // No new events will be added (an intersect => Done)
#     while (e = Eq.next()) {         // while there are events
#         if (e->type == LEFT) {      // process a left vertex
#             s = SL.add(e);          // add it to the sweep line
#             if (SL.intersect(  s, s->above))
#                  return FALSE;      // Pn is NOT simple
#             if (SL.intersect(  s, s->below))
#                  return FALSE;      // Pn is NOT simple
#         }
#         else {                      // processs a right vertex
#             s = SL.find(e);
#             if (SL.intersect(  s->above, s->below))
#                  return FALSE;      // Pn is NOT simple
#             SL.remove(s);           // remove it from the sweep line
#         }
#     }
#     return TRUE;      // Pn IS simple
# }
# //===================================================================