import math
from typing import List, Dict, Set
from vision.detection_data import RobotDetection

class RobotTracker:
    """
    Tracks robots across frames and assigns persistent IDs
    IDs are reused when robots disappear
    """
    
    def __init__(self, max_distance: float = 100.0, max_missing_frames: int = 15):
        """
        Args:
            max_distance: Maximum pixel distance to match detections between frames
            max_missing_frames: How many frames a robot can be missing before ID is freed
        """
        self.max_distance = max_distance
        self.max_missing_frames = max_missing_frames
        
        # Tracking state for red robots
        self.red_tracks: Dict[int, dict] = {}  # track_id -> {detection, missing_count}
        self.red_next_id = 1
        self.red_free_ids: Set[int] = set()  # Pool of IDs that can be reused
        
        # Tracking state for blue robots
        self.blue_tracks: Dict[int, dict] = {}
        self.blue_next_id = 1
        self.blue_free_ids: Set[int] = set()
    
    def _get_next_id(self, is_red: bool) -> int:
        """Get next available ID (reuse freed IDs first)"""
        if is_red:
            if self.red_free_ids:
                return min(self.red_free_ids)  # Reuse lowest freed ID
            else:
                id_val = self.red_next_id
                self.red_next_id += 1
                return id_val
        else:
            if self.blue_free_ids:
                return min(self.blue_free_ids)
            else:
                id_val = self.blue_next_id
                self.blue_next_id += 1
                return id_val
    
    def _free_id(self, track_id: int, is_red: bool):
        """Mark an ID as available for reuse"""
        if is_red:
            self.red_free_ids.add(track_id)
        else:
            self.blue_free_ids.add(track_id)
    
    def _distance(self, det1: RobotDetection, det2: RobotDetection) -> float:
        """Calculate Euclidean distance between two detections"""
        dx = det1.center_x - det2.center_x
        dy = det1.center_y - det2.center_y
        return math.sqrt(dx*dx + dy*dy)
    
    def _match_detections(self, new_detections: List[RobotDetection], 
                          existing_tracks: Dict[int, dict]) -> Dict[int, RobotDetection]:
        """
        Match new detections to existing tracks
        Returns: dict mapping track_id -> matched_detection
        """
        matches = {}
        used_detections = set()
        
        # For each existing track, find closest new detection
        for track_id, track_data in existing_tracks.items():
            old_detection = track_data['detection']
            best_match = None
            best_distance = float('inf')
            
            for i, new_det in enumerate(new_detections):
                if i in used_detections:
                    continue
                
                dist = self._distance(old_detection, new_det)
                if dist < best_distance and dist < self.max_distance:
                    best_distance = dist
                    best_match = i
            
            if best_match is not None:
                matches[track_id] = new_detections[best_match]
                used_detections.add(best_match)
        
        return matches, used_detections
    
    def update(self, detections: List[RobotDetection]) -> List[RobotDetection]:
        """
        Update tracking with new detections
        Returns: detections with track_id assigned
        """
        # Separate by color
        red_detections = [d for d in detections if d.is_red]
        blue_detections = [d for d in detections if d.is_blue]
        
        # Update red robots
        red_matched, red_used = self._match_detections(red_detections, self.red_tracks)
        
        # Update existing tracks
        new_red_tracks = {}
        for track_id, detection in red_matched.items():
            detection.track_id = track_id
            new_red_tracks[track_id] = {
                'detection': detection,
                'missing_count': 0
            }
        
        # Increment missing count for unmatched tracks
        for track_id, track_data in self.red_tracks.items():
            if track_id not in red_matched:
                track_data['missing_count'] += 1
                if track_data['missing_count'] <= self.max_missing_frames:
                    # Keep the track alive for a bit
                    new_red_tracks[track_id] = track_data
                else:
                    # Free the ID for reuse
                    self._free_id(track_id, is_red=True)
        
        # Create new tracks for unmatched detections
        for i, detection in enumerate(red_detections):
            if i not in red_used:
                new_id = self._get_next_id(is_red=True)
                detection.track_id = new_id
                # Remove from free pool if it was there
                self.red_free_ids.discard(new_id)
                new_red_tracks[new_id] = {
                    'detection': detection,
                    'missing_count': 0
                }
        
        self.red_tracks = new_red_tracks
        
        # Update blue robots (same logic)
        blue_matched, blue_used = self._match_detections(blue_detections, self.blue_tracks)
        
        new_blue_tracks = {}
        for track_id, detection in blue_matched.items():
            detection.track_id = track_id
            new_blue_tracks[track_id] = {
                'detection': detection,
                'missing_count': 0
            }
        
        for track_id, track_data in self.blue_tracks.items():
            if track_id not in blue_matched:
                track_data['missing_count'] += 1
                if track_data['missing_count'] <= self.max_missing_frames:
                    new_blue_tracks[track_id] = track_data
                else:
                    self._free_id(track_id, is_red=False)
        
        for i, detection in enumerate(blue_detections):
            if i not in blue_used:
                new_id = self._get_next_id(is_red=False)
                detection.track_id = new_id
                self.blue_free_ids.discard(new_id)
                new_blue_tracks[new_id] = {
                    'detection': detection,
                    'missing_count': 0
                }
        
        self.blue_tracks = new_blue_tracks
        
        return detections
    
    def reset(self):
        """Reset all tracking state"""
        self.red_tracks.clear()
        self.blue_tracks.clear()
        self.red_free_ids.clear()
        self.blue_free_ids.clear()
        self.red_next_id = 1
        self.blue_next_id = 1
    
    def get_active_ids(self) -> dict:
        """Get currently active track IDs"""
        return {
            'red': list(self.red_tracks.keys()),
            'blue': list(self.blue_tracks.keys())
        }
