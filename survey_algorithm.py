import xml.etree.ElementTree as ET
import math
import numpy as np
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
import argparse

class DroneFlightPathGenerator:
    def __init__(self, ground_coverage_per_frame: float = 1.0, overlap_percentage: float = 20, 
                 altitude: float = 100, drone_frame_size: float = 1.0, safety_margin: float = 2.0):
        """
        Initialize the drone flight path generator.
        
        Args:
            ground_coverage_per_frame: Ground coverage per frame in meters (1m x 1m)
            overlap_percentage: Percentage overlap between images (default: 20%)
            altitude: Drone altitude in meters (default: 100m)
            drone_frame_size: Drone frame size in meters (default: 1.0m for 1m x 1m frame)
            safety_margin: Safety margin multiplier for drone frame (default: 2.0x frame size)
        """
        self.ground_coverage_per_frame = ground_coverage_per_frame
        self.overlap_percentage = overlap_percentage
        self.altitude = altitude
        self.drone_frame_size = drone_frame_size
        self.safety_margin = safety_margin
        self.effective_safety_distance = drone_frame_size * safety_margin
        
        # Calculate camera FOV based on ground coverage and altitude
        self.camera_fov = self._calculate_fov_from_coverage()
        
    def _calculate_fov_from_coverage(self) -> float:
        """Calculate camera FOV from ground coverage and altitude."""
        # For square coverage: ground_width = 2 * altitude * tan(fov/2)
        # Therefore: fov = 2 * arctan(ground_width / (2 * altitude))
        fov_rad = 2 * math.atan(self.ground_coverage_per_frame / (2 * self.altitude))
        return math.degrees(fov_rad)
        
    def extract_coordinates_from_kml(self, kml_file_path: str) -> List[List[Tuple[float, float]]]:
        """
        Extract coordinates from KML file.
        
        Args:
            kml_file_path: Path to the KML file
            
        Returns:
            List of polygons, each containing list of (lon, lat) tuples
        """
        try:
            tree = ET.parse(kml_file_path)
            root = tree.getroot()
            
            # Handle KML namespace
            namespace = {'kml': 'http://www.opengis.net/kml/2.2'}
            if root.tag.startswith('{'):
                ns_uri = root.tag[root.tag.find("{")+1:root.tag.find("}")]
                namespace = {'kml': ns_uri}
            
            polygons = []
            
            # Find all coordinate elements
            for elem in root.iter():
                if elem.tag.endswith('coordinates') or elem.tag == 'coordinates':
                    coord_text = elem.text.strip()
                    if coord_text:
                        polygon_coords = self._parse_coordinates(coord_text)
                        if polygon_coords:
                            polygons.append(polygon_coords)
            
            return polygons
            
        except ET.ParseError as e:
            print(f"Error parsing KML file: {e}")
            return []
        except FileNotFoundError:
            print(f"KML file not found: {kml_file_path}")
            return []
    
    def _parse_coordinates(self, coord_text: str) -> List[Tuple[float, float]]:
        """Parse coordinate text from KML into list of (lon, lat) tuples."""
        coordinates = []
        
        # Split by whitespace and filter empty strings
        coord_parts = [part.strip() for part in coord_text.replace('\n', ' ').split() if part.strip()]
        
        for part in coord_parts:
            # KML coordinates are in format: longitude,latitude,altitude (altitude optional)
            coords = part.split(',')
            if len(coords) >= 2:
                try:
                    lon = float(coords[0])
                    lat = float(coords[1])
                    coordinates.append((lon, lat))
                except ValueError:
                    continue
                    
        return coordinates
    
    def calculate_coverage_area(self) -> float:
        """
        Calculate the ground coverage area of the camera at given altitude.
        
        Returns:
            Coverage area in square meters
        """
        return self.ground_coverage_per_frame * self.ground_coverage_per_frame
    
    def calculate_waypoint_spacing(self) -> float:
        """
        Calculate spacing between waypoints based on overlap percentage.
        
        Returns:
            Spacing in meters
        """
        # Calculate spacing with overlap
        overlap_factor = (100 - self.overlap_percentage) / 100
        base_spacing = self.ground_coverage_per_frame * overlap_factor
        
        # Ensure minimum spacing for drone frame safety
        min_spacing = self.effective_safety_distance
        
        return max(base_spacing, min_spacing)
    
    def degrees_to_meters(self, lat: float) -> Tuple[float, float]:
        """
        Convert degrees to meters at given latitude.
        
        Args:
            lat: Latitude in degrees
            
        Returns:
            Tuple of (meters_per_degree_lon, meters_per_degree_lat)
        """
        # Approximate conversion (more accurate methods available for precision work)
        meters_per_degree_lat = 111319.9  # Constant
        meters_per_degree_lon = 111319.9 * math.cos(math.radians(lat))
        
        return meters_per_degree_lon, meters_per_degree_lat
    
    def generate_zigzag_pattern(self, polygon_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Generate a zigzag (boustrophedon) pattern of waypoints to cover the polygon area completely.
        
        Args:
            polygon_coords: List of (lon, lat) coordinates defining the boundary
            
        Returns:
            List of waypoints as (lon, lat) tuples in zigzag order
        """
        if len(polygon_coords) < 3:
            return []
        
        # Create shapely polygon
        polygon = Polygon(polygon_coords)
        
        # Create a buffer inside the polygon to account for drone frame size and safety margin
        # Negative buffer to shrink the polygon inward
        safety_buffer_degrees = self._meters_to_degrees_approx(
            self.effective_safety_distance, 
            polygon.centroid.y
        )
        
        # Apply safety buffer (shrink polygon inward)
        buffered_polygon = polygon.buffer(-safety_buffer_degrees)
        
        # Handle case where buffer makes polygon too small or invalid
        if buffered_polygon.is_empty or not buffered_polygon.is_valid:
            print(f"Warning: Safety buffer ({self.effective_safety_distance:.1f}m) too large for polygon. Using original polygon.")
            buffered_polygon = polygon
            safety_buffer_degrees = 0
        
        # Get bounding box of buffered polygon
        min_lon, min_lat, max_lon, max_lat = buffered_polygon.bounds
        
        # Calculate center latitude for conversion
        center_lat = (min_lat + max_lat) / 2
        meters_per_deg_lon, meters_per_deg_lat = self.degrees_to_meters(center_lat)
        
        # Calculate waypoint spacing - use slightly tighter spacing to ensure complete coverage
        spacing_meters = self.calculate_waypoint_spacing()
        spacing_lon = spacing_meters / meters_per_deg_lon
        spacing_lat = spacing_meters / meters_per_deg_lat
        
        # Generate zigzag pattern
        waypoints = []
        current_lat = min_lat
        row = 0
        
        # Store all valid points first, then sort them into zigzag pattern
        all_valid_points = []
        
        while current_lat <= max_lat:
            row_points = []
            current_lon = min_lon
            
            # Collect all valid points in this row
            while current_lon <= max_lon:
                point = Point(current_lon, current_lat)
                
                # Check if point is inside buffered polygon or very close to boundary
                if (buffered_polygon.contains(point) or 
                    buffered_polygon.boundary.distance(point) < spacing_lon/4):
                    row_points.append((current_lon, current_lat, row))
                
                current_lon += spacing_lon
            
            # Add row points to all valid points
            if row_points:
                all_valid_points.append((row, row_points))
            
            current_lat += spacing_lat
            row += 1
        
        # Create zigzag pattern from valid points
        for row_num, row_points in all_valid_points:
            if row_num % 2 == 0:  # Even rows: left to right
                row_points.sort(key=lambda p: p[0])  # Sort by longitude ascending
            else:  # Odd rows: right to left
                row_points.sort(key=lambda p: p[0], reverse=True)  # Sort by longitude descending
            
            # Add points to waypoints list (exclude row number from tuple)
            for lon, lat, _ in row_points:
                waypoints.append((lon, lat))
        
        return waypoints
    
    def _meters_to_degrees_approx(self, meters: float, latitude: float) -> float:
        """
        Convert meters to degrees at given latitude (approximate).
        
        Args:
            meters: Distance in meters
            latitude: Latitude in degrees
            
        Returns:
            Approximate distance in degrees
        """
        meters_per_deg_lat = 111319.9
        return meters / meters_per_deg_lat
    
    def calculate_flight_statistics(self, waypoints: List[Tuple[float, float]]) -> dict:
        """
        Calculate comprehensive flight statistics.
        
        Args:
            waypoints: List of waypoints
            
        Returns:
            Dictionary containing flight statistics
        """
        if not waypoints:
            return {}
        
        # Calculate total flight distance
        total_distance = 0.0
        if len(waypoints) > 1:
            for i in range(len(waypoints) - 1):
                lon1, lat1 = waypoints[i]
                lon2, lat2 = waypoints[i + 1]
                
                # Approximate distance calculation
                center_lat = (lat1 + lat2) / 2
                meters_per_deg_lon, meters_per_deg_lat = self.degrees_to_meters(center_lat)
                
                dx = (lon2 - lon1) * meters_per_deg_lon
                dy = (lat2 - lat1) * meters_per_deg_lat
                distance = math.sqrt(dx*dx + dy*dy)
                total_distance += distance
        
        # Calculate estimated flight time (assuming 5 m/s average speed)
        avg_speed_ms = 5.0  # m/s
        flight_time_seconds = total_distance / avg_speed_ms
        flight_time_minutes = flight_time_seconds / 60.0
        
        # Calculate coverage statistics
        coverage_per_frame = self.calculate_coverage_area()
        total_coverage = len(waypoints) * coverage_per_frame
        
        return {
            'total_waypoints': len(waypoints),
            'total_distance_m': total_distance,
            'estimated_flight_time_min': flight_time_minutes,
            'coverage_per_frame_m2': coverage_per_frame,
            'total_theoretical_coverage_m2': total_coverage,
            'waypoint_spacing_m': self.calculate_waypoint_spacing(),
            'ground_coverage_per_frame_m': self.ground_coverage_per_frame,
            'overlap_percentage': self.overlap_percentage,
            'calculated_fov_deg': self.camera_fov
        }
    
    def generate_flight_path(self, kml_file_path: str) -> List[Tuple[float, float]]:
        """
        Generate complete flight path from KML file.
        
        Args:
            kml_file_path: Path to KML file
            
        Returns:
            List of waypoints as (lon, lat) tuples
        """
        polygons = self.extract_coordinates_from_kml(kml_file_path)
        
        if not polygons:
            print("No valid polygons found in KML file")
            return []
        
        all_waypoints = []
        
        for i, polygon_coords in enumerate(polygons):
            print(f"Processing polygon {i+1} with {len(polygon_coords)} vertices")
            waypoints = self.generate_zigzag_pattern(polygon_coords)
            all_waypoints.extend(waypoints)
            print(f"Generated {len(waypoints)} waypoints for polygon {i+1}")
        
        return all_waypoints
    
    def visualize_flight_path(self, kml_file_path: str, waypoints: List[Tuple[float, float]]) -> None:
        """
        Visualize the flight path with original polygons and flight statistics.
        
        Args:
            kml_file_path: Path to KML file
            waypoints: List of waypoints to visualize
        """
        polygons = self.extract_coordinates_from_kml(kml_file_path)
        stats = self.calculate_flight_statistics(waypoints)
        
        plt.figure(figsize=(14, 10))
        
        # Plot original polygons as reference
        for i, polygon_coords in enumerate(polygons):
            if len(polygon_coords) >= 3:
                closed_coords = polygon_coords + [polygon_coords[0]]
                lons, lats = zip(*closed_coords)
                plt.plot(lons, lats, 'b-', linewidth=2, alpha=0.8, 
                        label=f'Area Boundary {i+1}' if i == 0 else "")
                plt.fill(lons, lats, 'blue', alpha=0.15)
        
        # Plot zigzag flight path
        if waypoints:
            way_lons, way_lats = zip(*waypoints)
            
            # Plot flight path lines with zigzag pattern clearly visible
            plt.plot(way_lons, way_lats, 'r-', alpha=0.8, linewidth=2, label='Zigzag Flight Path')
            
            # Plot waypoints
            plt.scatter(way_lons, way_lats, c='red', s=20, alpha=0.9, 
                       label=f'Waypoints ({len(waypoints)})', zorder=5, edgecolors='darkred', linewidths=0.5)
            
            # Mark start and end points
            plt.scatter(way_lons[0], way_lats[0], c='green', s=120, marker='^', 
                       label='Start Point', zorder=6, edgecolors='darkgreen', linewidths=2)
            plt.scatter(way_lons[-1], way_lats[-1], c='orange', s=120, marker='s', 
                       label='End Point', zorder=6, edgecolors='darkorange', linewidths=2)
            
            # Add direction arrows every 10th waypoint to show zigzag pattern
            arrow_interval = max(1, len(waypoints) // 20)  # Show ~20 arrows max
            for i in range(0, len(waypoints) - arrow_interval, arrow_interval):
                x1, y1 = way_lons[i], way_lats[i]
                x2, y2 = way_lons[i + arrow_interval], way_lats[i + arrow_interval]
                
                dx = x2 - x1
                dy = y2 - y1
                
                plt.annotate('', xy=(x2, y2), xytext=(x1, y1),
                           arrowprops=dict(arrowstyle='->', color='purple', lw=1.5, alpha=0.7))
        
        plt.xlabel('Longitude', fontsize=12)
        plt.ylabel('Latitude', fontsize=12)
        
        # Enhanced title with key parameters
        title = f'Drone Zigzag Flight Plan - Complete Area Coverage\n'
        title += f'Frame Coverage: {self.ground_coverage_per_frame}m × {self.ground_coverage_per_frame}m | '
        title += f'Altitude: {self.altitude}m | Overlap: {self.overlap_percentage}% | '
        title += f'Safety Distance: {self.effective_safety_distance:.1f}m'
        plt.title(title, fontsize=14, fontweight='bold')
        
        plt.legend(loc='upper left', bbox_to_anchor=(1.02, 1))
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # Enhanced flight statistics
        if stats:
            stats_text = f"""FLIGHT STATISTICS
Waypoints: {stats['total_waypoints']}
Flight Distance: {stats['total_distance_m']:.0f}m ({stats['total_distance_m']/1000:.1f}km)
Est. Flight Time: {stats['estimated_flight_time_min']:.1f} min
Coverage/Frame: {stats['coverage_per_frame_m2']:.1f}m²
Waypoint Spacing: {stats['waypoint_spacing_m']:.1f}m
Calculated FOV: {stats['calculated_fov_deg']:.1f}°

COVERAGE DETAILS
Ground Coverage: {self.ground_coverage_per_frame}m × {self.ground_coverage_per_frame}m
Overlap: {self.overlap_percentage}%
Safety Buffer: {self.effective_safety_distance:.1f}m
Total Images: {stats['total_waypoints']}"""
            
            plt.figtext(0.02, 0.02, stats_text, fontsize=10, 
                       bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue", alpha=0.9),
                       verticalalignment='bottom')
        
        plt.tight_layout()
        plt.show()
    
    def export_waypoints_to_csv(self, waypoints: List[Tuple[float, float]], output_file: str) -> None:
        """
        Export waypoints to CSV file with enhanced information.
        
        Args:
            waypoints: List of waypoints
            output_file: Output CSV file path
        """
        import csv
        
        stats = self.calculate_flight_statistics(waypoints)
        
        with open(output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header with metadata
            writer.writerow(['# Drone Flight Plan - Zigzag Pattern'])
            writer.writerow([f'# Generated waypoints: {len(waypoints)}'])
            writer.writerow([f'# Ground coverage per frame: {self.ground_coverage_per_frame}m x {self.ground_coverage_per_frame}m'])
            writer.writerow([f'# Altitude: {self.altitude}m'])
            writer.writerow([f'# Overlap: {self.overlap_percentage}%'])
            writer.writerow([f'# Safety distance: {self.effective_safety_distance:.1f}m'])
            if stats:
                writer.writerow([f'# Total flight distance: {stats["total_distance_m"]:.0f}m'])
                writer.writerow([f'# Estimated flight time: {stats["estimated_flight_time_min"]:.1f} minutes'])
            writer.writerow([''])
            
            # Write data header
            writer.writerow(['Waypoint', 'Longitude', 'Latitude', 'Altitude_m', 'Sequence'])
            
            for i, (lon, lat) in enumerate(waypoints, 1):
                writer.writerow([i, f'{lon:.8f}', f'{lat:.8f}', self.altitude, f'WP_{i:04d}'])
        
        print(f"Exported {len(waypoints)} waypoints to {output_file}")
        if stats:
            print(f"Total flight distance: {stats['total_distance_m']:.0f}m ({stats['total_distance_m']/1000:.2f}km)")
            print(f"Estimated flight time: {stats['estimated_flight_time_min']:.1f} minutes")

def main():
    parser = argparse.ArgumentParser(description='Generate drone zigzag flight path from KML file')
    parser.add_argument('kml_file', nargs='?', default=None, help='Path to KML file')
    parser.add_argument('--altitude', type=float, default=100, help='Drone altitude in meters (default: 100)')
    parser.add_argument('--coverage', type=float, default=1.0, help='Ground coverage per frame in meters (default: 1.0 for 1m×1m)')
    parser.add_argument('--overlap', type=float, default=20, help='Image overlap percentage (default: 20)')
    parser.add_argument('--drone-frame', type=float, default=1.0, help='Drone frame size in meters (default: 1.0)')
    parser.add_argument('--safety-margin', type=float, default=2.0, help='Safety margin multiplier for drone frame (default: 2.0)')
    parser.add_argument('--output', type=str, help='Output CSV file for waypoints')
    parser.add_argument('--visualize', action='store_true', help='Show flight path visualization')
    
    args = parser.parse_args()
    
    # If no KML file provided via command line, use hardcoded path
    kml_file_path = args.kml_file
    
    if not kml_file_path:
        # OPTION 1: Hardcode your file path here
        kml_file_path = r"College_playground_5m_altitude"
        
        # OPTION 2: Uncomment to prompt user for file path
        # kml_file_path = input("Enter path to KML file: ").strip().strip('"')
    
    print(f"Processing KML file: {kml_file_path}")
    print(f"Ground coverage per frame: {args.coverage}m × {args.coverage}m")
    
    # Create flight path generator
    generator = DroneFlightPathGenerator(
        ground_coverage_per_frame=args.coverage,
        overlap_percentage=args.overlap,
        altitude=args.altitude,
        drone_frame_size=args.drone_frame,
        safety_margin=args.safety_margin
    )
    
    # Generate flight path
    waypoints = generator.generate_flight_path(kml_file_path)
    
    if waypoints:
        stats = generator.calculate_flight_statistics(waypoints)
        
        print(f"\n{'='*50}")
        print("FLIGHT PLAN GENERATED SUCCESSFULLY")
        print(f"{'='*50}")
        print(f"Total waypoints: {len(waypoints)}")
        print(f"Coverage per frame: {args.coverage}m × {args.coverage}m ({generator.calculate_coverage_area():.1f} m²)")
        print(f"Calculated FOV: {generator.camera_fov:.1f}°")
        print(f"Waypoint spacing: {generator.calculate_waypoint_spacing():.1f}m")
        print(f"Safety distance: {generator.effective_safety_distance:.1f}m")
        
        if stats:
            print(f"Total flight distance: {stats['total_distance_m']:.0f}m ({stats['total_distance_m']/1000:.2f}km)")
            print(f"Estimated flight time: {stats['estimated_flight_time_min']:.1f} minutes")
            print(f"Total theoretical coverage: {stats['total_theoretical_coverage_m2']:.0f} m²")
        
        # Export to CSV if requested or use default name
        output_file = args.output if args.output else "drone_zigzag_waypoints.csv"
        generator.export_waypoints_to_csv(waypoints, output_file)
        
        # Show visualization by default or if requested
        if args.visualize or True:  # Always show visualization
            generator.visualize_flight_path(kml_file_path, waypoints)
            
        # Print first few waypoints as example
        print(f"\nFirst 10 waypoints (Zigzag Pattern):")
        for i, (lon, lat) in enumerate(waypoints[:10], 1):
            print(f"  WP_{i:04d}: {lon:.8f}, {lat:.8f}")
            
    else:
        print("No waypoints generated. Check your KML file and parameters.")

def run_quick_demo():
    """
    Quick demo function with optimal settings for 1m×1m coverage
    """
    # YOUR KML FILE PATH HERE
    kml_file_path = r"C:\Users\Abhinav S D\OneDrive\Desktop\aerocet 2024-25 annual report\ground area1.kml"
    
    print("=== DRONE ZIGZAG FLIGHT PLANNER ===")
    print("Optimized for 1m × 1m ground coverage per frame")
    print()
    
    # Create flight path generator with optimal settings for 1m×1m coverage
    generator = DroneFlightPathGenerator(
        ground_coverage_per_frame=1.0,  # 1m × 1m coverage per frame
        overlap_percentage=20,          # 20% overlap for good coverage
        altitude=100,                   # 100m altitude
        drone_frame_size=1.0,          # 1m × 1m drone frame
        safety_margin=2.0              # 2m safety distance
    )
    
    print(f"Processing: {kml_file_path}")
    print(f"Settings: 1m×1m coverage, 20% overlap, 100m altitude")
    print(f"Calculated FOV: {generator.camera_fov:.1f}°")
    print()
    
    # Generate zigzag flight path
    waypoints = generator.generate_flight_path(kml_file_path)
    
    if waypoints:
        stats = generator.calculate_flight_statistics(waypoints)
        
        print("✅ FLIGHT PLAN READY!")
        print(f"📍 Total waypoints: {len(waypoints)}")
        print(f"📏 Flight distance: {stats['total_distance_m']:.0f}m ({stats['total_distance_m']/1000:.2f}km)")
        print(f"⏱  Estimated time: {stats['estimated_flight_time_min']:.1f} minutes")
        print(f"📐 Waypoint spacing: {stats['waypoint_spacing_m']:.1f}m")
        print()
        
        # Export waypoints
        output_file = "drone_zigzag_flight_plan.csv"
        generator.export_waypoints_to_csv(waypoints, output_file)
        
        # Show visualization
        print("🎨 Showing flight path visualization...")
        generator.visualize_flight_path(kml_file_path, waypoints)
        
        print(f"✅ Complete! Check '{output_file}' for waypoint details.")
        
    else:
        print("❌ No waypoints generated. Please check your KML file.")

if __name__ == "_main_":
    # Choose one option:
    
    # OPTION A: Full command line interface
    main()
    
    # OPTION B: Quick demo (uncomment and comment out main() above)
    # run_quick_demo()
