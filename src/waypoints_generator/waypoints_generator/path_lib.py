import math

class Point:
    """
    A simple class to represent a 2D point and perform basic vector operations.
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"({self.x:.4f}, {self.y:.4f})" # Format for cleaner output

    def __add__(self, other):
        """Vector addition."""
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        """Vector subtraction."""
        return Point(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        """Scalar multiplication."""
        return Point(self.x * scalar, self.y * scalar)

    def magnitude(self):
        """Calculates the magnitude (length) of the vector."""
        return math.sqrt(self.x**2 + self.y**2)

    def normalize(self):
        """Normalizes the vector to a unit vector."""
        mag = self.magnitude()
        if math.isclose(mag, 0.0):
            # Handle zero vector case to prevent division by zero
            return Point(0, 0)
        return Point(self.x / mag, self.y / mag)

    def perp_ccw(self):
        """
        Returns a vector perpendicular to the current vector, rotated 90 degrees
        counter-clockwise. For a counter-clockwise polygon, this points inward
        relative to an edge vector.
        """
        return Point(-self.y, self.x)





class path_generator():
    def __init__(self,vertics, road_width, safe_distance,step_length):
        self.vertics = vertics
        self.road_width = road_width 
        self.safe_distance = safe_distance
        self.step_lenght = step_length

    def cal_distance(self,p1:Point,p2:Point)->float:
        distance = (p1.x - p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)

        return distance

    def cal_theta(self,p1:Point, p2:Point):
        # print(p1)
        val = (p2.y-p1.y)/(p2.x-p1.x)

        return math.atan(val)

    def line_intersection(self,p1, p2, p3, p4):
        """
        Finds the intersection point of two lines defined by (p1, p2) and (p3, p4).
        Returns a Point object if an intersection exists, None if lines are parallel or coincident.
        """
        # Line 1: A1*x + B1*y = C1
        A1 = p2.y - p1.y
        B1 = p1.x - p2.x
        C1 = A1 * p1.x + B1 * p1.y

        # Line 2: A2*x + B2*y = C2
        A2 = p4.y - p3.y
        B2 = p3.x - p4.x
        C2 = A2 * p3.x + B2 * p3.y

        determinant = A1 * B2 - A2 * B1

        # If determinant is close to zero, lines are parallel or coincident
        if math.isclose(determinant, 0.0):
            return None
        else:
            x = (B2 * C1 - B1 * C2) / determinant
            y = (A1 * C2 - A2 * C1) / determinant
            return Point(x, y)

    def inner_bound_vertics(self,vertices,d):
        """
        Shrinks a polygon inwards by a given distance d.

        Args:
            vertices (list of tuple): A list of (x, y) tuples representing the
                                    polygon vertices in counter-clockwise order.
            d (float): The distance by which to shrink the polygon inwards. Must be positive.

        Returns:
            list of tuple: A new list of (x, y) tuples representing the vertices
                        of the shrunk polygon. Returns an empty list if the
                        polygon collapses or is invalid.
        """
        n = len(vertices)
        if n < 3:
            print("Error: A polygon must have at least 3 vertices.")
            return []
        if d < 0:
            print("Error: Shrink distance 'd' must be positive. Use a positive value for shrinking.")
            return []

        # Convert input tuples to Point objects for easier vector math
        points = [Point(v[0], v[1]) for v in vertices]
        shrunk_points = []

        for i in range(n):
            # Get the previous, current, and next vertices
            p_prev = points[(i - 1 + n) % n]
            p_curr = points[i]
            p_next = points[(i + 1) % n]

            # Calculate the offset line for the previous edge (p_prev -> p_curr)
            edge_vec_prev = p_curr - p_prev
            if math.isclose(edge_vec_prev.magnitude(), 0.0):
                print(f"Warning: Degenerate edge detected at vertex {p_curr}. Skipping.")
                continue # Skip this vertex if the edge is degenerate

            # The inward normal for a CCW polygon edge is its perpendicular CCW vector
            unit_normal_prev = edge_vec_prev.perp_ccw().normalize()
            offset_vec_prev = unit_normal_prev * d
            line1_p1 = p_prev + offset_vec_prev
            line1_p2 = p_curr + offset_vec_prev

            # Calculate the offset line for the current edge (p_curr -> p_next)
            edge_vec_curr = p_next - p_curr
            if math.isclose(edge_vec_curr.magnitude(), 0.0):
                print(f"Warning: Degenerate edge detected at vertex {p_next}. Skipping.")
                continue # Skip this vertex if the edge is degenerate

            unit_normal_curr = edge_vec_curr.perp_ccw().normalize()
            offset_vec_curr = unit_normal_curr * d
            line2_p1 = p_curr + offset_vec_curr
            line2_p2 = p_next + offset_vec_curr
            print(line1_p1)
            print(line1_p2)

            # The new vertex is the intersection of these two offset lines
            new_vertex = self.line_intersection(line1_p1, line1_p2, line2_p1, line2_p2)

            if new_vertex is None:
                # If lines are parallel or coincident, the polygon likely collapses or folds
                print(f"Warning: Polygon may collapse or become invalid for d={d}. Offset lines are parallel or coincident near vertex {p_curr}.")
                return [] # Return an empty list to indicate collapse/invalidity
            
            shrunk_points.append(new_vertex)
        
        # Convert Point objects back to tuples for the output
        return [(p.x, p.y) for p in shrunk_points]
        
    def search_intersection_with_gap(self, vertices, gap):
    # print(gap)
        points = [Point(v[0], v[1]) for v in vertices]
        scan_line_p1 = points[0]
        ver_size = len(points)
        ndone = True
        scan_line_p2 = points[(- 1 + ver_size) % ver_size]
        scan_line_vec = scan_line_p1 - scan_line_p2
        unit_normal_scan = scan_line_vec.perp_ccw().normalize()
        n = 1
        vertices_id_top = 0
        vertices_id_bot = 0
        intersect_pts = []  
        update_top = True
        update_bottom = True
        
        intersect__top = []
        intersect_bottom = []


        ## current issue : scan line donest sync -> fix the n value increament logic 
        while ndone:
            # print(n)

            ## creating the scan line 
            offset_vec_scan = unit_normal_scan * gap * n

            ## shift the scan line towards right each step 
            shift_scan_pt_1 = scan_line_p1 + offset_vec_scan
            shift_scan_pt_2 = scan_line_p2 + offset_vec_scan
            # print("shift pt: ", shift_scan_pt_1)

            ## update top or bottom vertics if needed 
            if update_top:
                top_point_1 = points[(ver_size-1-vertices_id_top) % ver_size]
                top_point_2 = points[(ver_size-2-vertices_id_top) % ver_size]
                top_distance = self.cal_distance(points[0],top_point_2)
                update_top = False

            if update_bottom :    
                bot_point_1 = points[vertices_id_bot]
                bot_point_2 = points[vertices_id_bot+1]
                bot_distance = self.cal_distance(points[0],bot_point_2)
                update_bottom = False

            # ## if top and bot meet, return the list 
            # print("top_point_2 : " ,top_point_2)
            # print("bot_point_2 : ", bot_point_2)
            if top_point_2 == bot_point_2:

                top_list = [(p.x, p.y) for p in intersect__top]
                bot_list = [(p.x, p.y) for p in intersect_bottom]    
                
                return top_list, bot_list 
                # return [(p.x, p.y) for p in intersect_pts]

            ##cal the intersection pts 
            top_intersect_pt = self.line_intersection(shift_scan_pt_1,shift_scan_pt_2,top_point_1,top_point_2)
            bot_intersect_pt = self.line_intersection(shift_scan_pt_1,shift_scan_pt_2,bot_point_1,bot_point_2)

            """ 
            ---update list logic---
            1. If both pt are not None and both in range, then a pair found, put them into list by compare angle 
                update n to swipe to right 
            
            2. If both pt are not None, and one of pt not in range, shift the vertics 

            3. If one or both are None, then shift the vertics (parallel case)
            
            """
            # print("top_insersect_pt : ", top_intersect_pt)
            # print("bot_intersect_pt : ", bot_intersect_pt)
            if top_intersect_pt and bot_intersect_pt : 
                top_intersect_dis = self.cal_distance(points[0],top_intersect_pt)
                bot_intersect_dis = self.cal_distance(points[0],bot_intersect_pt)
                top_angle = self.cal_theta(points[0],top_intersect_pt)
                bot_angle = self.cal_theta(points[0],bot_intersect_pt)

                if top_intersect_dis < top_distance and bot_intersect_dis < bot_distance : 
                    # print("both in range, a pair found")

                    if top_angle > bot_angle: 
                        intersect__top.append(top_intersect_pt)
                        intersect_pts.append(top_intersect_pt)
                        intersect_bottom.append(bot_intersect_pt)
                        intersect_pts.append(bot_intersect_pt)
                    else : 
                        intersect__top.append(top_intersect_pt)
                        intersect_pts.append(bot_intersect_pt)
                        intersect_bottom.append(top_intersect_pt)
                        intersect_pts.append(top_intersect_pt)
                    # move to next scan 
                    n = n + 1 

                elif top_intersect_dis >= top_distance or bot_intersect_dis >= bot_distance:
                    # print("one of pt out of the range, update the vertics")

                    if top_intersect_dis >= top_distance : 
                        update_top = True 
                        vertices_id_top = vertices_id_top + 1 
                    
                    elif bot_intersect_dis >= bot_distance : 
                        update_bottom = True 
                        vertices_id_bot = vertices_id_bot + 1 

            if top_intersect_pt is None or bot_intersect_pt is None : 
                # print("One of the point is None, skip vertics instead")

                if top_intersect_pt is None : 
                    update_top = True 
                    vertices_id_top = vertices_id_top + 1 

                if bot_intersect_pt is None : 
                    update_bottom = True 
                    vertices_id_bot = vertices_id_bot + 1 

    def shuffle_list(self,top,bot):
        size_t = len(top)
        size_b = len(bot)

        new_list = [] 

        id_t = 0
        id_b = 1 

        new_list.append(bot[0])

        # new_list.append(top[0])

        # new_list.append(top[1])

        # new_list.append(bot[1])
        while id_t < size_t or id_b < size_b : 

            for i in range(2):
                if id_t < size_t:
                    new_list.append(top[id_t])
                    id_t = id_t + 1 
                else :
                    break
            
            for j in range(2):
                if id_b < size_b : 
                    new_list.append(bot[id_b])
                    id_b = id_b + 1 
                else : 
                    break 
            
        return new_list

    def add_station_pt(self, p_list, step):
        print("adding station points")        
        station_list = []
        size_l = len(p_list)
        iteration_id = 0 
        points = [Point(v[0], v[1]) for v in p_list]
        while iteration_id + 1 < size_l :
            tail = points[iteration_id]
            head = points[iteration_id + 1]
            distance_power = self.cal_distance(tail,head)
            distance = math.sqrt(distance_power)
            station_list.append(tail)
            if distance < step : 
                iteration_id = iteration_id + 1 
            else :
                while True:
                    vector = head - tail 
                    unit_vector = vector.normalize()
                    next_tail = tail + Point(step*unit_vector.x, step*unit_vector.y)
                    next_distance = math.sqrt(self.cal_distance(head,next_tail))
                    print(next_distance)
                    
                    if next_distance < step:
                        ##check if the next_point too close to the turning
                        if next_distance > 0.2:
                            station_list.append(next_tail)
                        iteration_id = iteration_id + 1 
                        break 
                    
                    station_list.append(next_tail)
                    tail = next_tail
        
        station_list.append(points[-1])
        return  [(p.x, p.y) for p in station_list]
    
    def generate_path(self):
        ##generate the inner bound first 
        inner_vertices = self.inner_bound_vertics(self.vertics,self.safe_distance)

        ##found top and bottom cross points 
        top, bot = self.search_intersection_with_gap(inner_vertices,self.road_width)

        ##rearrange two list into one 
        coverage_path = self.shuffle_list(top,bot)

        ##add sation points 
        final_coverage_path = self.add_station_pt(coverage_path,self.step_lenght)

        return final_coverage_path
