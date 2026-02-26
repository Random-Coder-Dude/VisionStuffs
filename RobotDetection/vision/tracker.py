import math
from typing import List, Dict, Set
from vision.detection_data import RobotDetection


class RobotTracker:
    """
    Assigns persistent IDs to robots across frames so the robot code can track
    "Robot RED1 moved left" rather than "there is a red robot somewhere."

    THE PROBLEM:
        detect() in detector.py produces a fresh list of RobotDetection objects
        every frame. Without tracking, you'd get RED robot at (x=100) one frame,
        then RED robot at (x=105) the next - but there's no way to know it's the
        same physical robot. IDs would flicker randomly.

    THE SOLUTION - Nearest Neighbor Matching:
        Each frame, we compare new detections against the tracks from last frame.
        For each existing track, we find the closest new detection (by pixel distance).
        If it's within max_distance, we call it the same robot and keep its ID.
        If no close match exists, the track survives for max_missing_frames frames
        before we give up and free its ID.
        New detections with no matching track get a new ID.

    ID REUSE:
        When a track expires (robot gone too long), its ID goes into a free pool.
        The next new robot gets the lowest available recycled ID rather than always
        incrementing, keeping IDs small and predictable.

    SEPARATE TRACKING PER COLOR:
        Red and blue robots are tracked independently. This prevents a scenario where
        a red robot at (100, 200) accidentally "matches" a blue robot at (105, 202).
        Each color has its own tracks dict, next_id counter, and free_ids pool.

    CALLED BY:
        BumperDetector.confirm_robots() in detector.py, once per frame after
        detections are created but before they're stored in latest_detections.
    """

    def __init__(self, max_distance: float = 100.0, max_missing_frames: int = 15):
        """
        max_distance:       Max pixel distance between frames to still consider
                            two detections the same robot. Tune this based on how
                            fast robots move between frames at your capture rate.
        max_missing_frames: How many consecutive frames a robot can be absent before
                            we give up on it and free its ID. Higher = more tolerant
                            of detector misses, lower = faster ID cleanup.
        """
        self.max_distance = max_distance
        self.max_missing_frames = max_missing_frames

        # Each entry: track_id -> {'detection': RobotDetection, 'missing_count': int}
        # missing_count increments each frame the robot isn't detected.
        # When missing_count > max_missing_frames, the track is deleted and ID freed.
        self.red_tracks: Dict[int, dict] = {}
        self.blue_tracks: Dict[int, dict] = {}

        # Next ID to assign if free pool is empty.
        # Starts at 1 so first robot is RED1 / BLUE1.
        self.red_next_id = 1
        self.blue_next_id = 1

        # Pool of IDs freed from expired tracks.
        # New robots get the lowest recycled ID first (min() of the set).
        self.red_free_ids: Set[int] = set()
        self.blue_free_ids: Set[int] = set()

    def _get_next_id(self, is_red: bool) -> int:
        """
        Returns the next available ID for a new track.
        Prefers recycled IDs (from expired tracks) over always-incrementing.
        """
        if is_red:
            if self.red_free_ids:
                return min(self.red_free_ids)  # Reuse lowest freed ID
            id_val = self.red_next_id
            self.red_next_id += 1
            return id_val
        else:
            if self.blue_free_ids:
                return min(self.blue_free_ids)
            id_val = self.blue_next_id
            self.blue_next_id += 1
            return id_val

    def _free_id(self, track_id: int, is_red: bool):
        """
        Returns an expired track's ID to the free pool so it can be reused.
        Called when a track's missing_count exceeds max_missing_frames.
        """
        if is_red:
            self.red_free_ids.add(track_id)
        else:
            self.blue_free_ids.add(track_id)

    def _distance(self, det1: RobotDetection, det2: RobotDetection) -> float:
        """
        Euclidean pixel distance between the centers of two detections.
        Used to decide if two detections across frames are the same robot.
        """
        dx = det1.center_x - det2.center_x
        dy = det1.center_y - det2.center_y
        return math.sqrt(dx * dx + dy * dy)

    def _match_detections(self, new_detections: List[RobotDetection],
                          existing_tracks: Dict[int, dict]):
        """
        Greedy nearest-neighbor matching: for each existing track, find the
        closest unmatched new detection within max_distance.

        Returns:
            matches:          dict of track_id -> matched RobotDetection
            used_detections:  set of indices into new_detections that were matched
                              (so we know which new detections need NEW IDs)
        """
        matches = {}
        used_detections = set()

        for track_id, track_data in existing_tracks.items():
            old_detection = track_data['detection']
            best_match = None
            best_distance = float('inf')

            for i, new_det in enumerate(new_detections):
                if i in used_detections:
                    continue  # Already claimed by another track
                dist = self._distance(old_detection, new_det)
                if dist < best_distance and dist < self.max_distance:
                    best_distance = dist
                    best_match = i

            if best_match is not None:
                matches[track_id] = new_detections[best_match]
                used_detections.add(best_match)

        return matches, used_detections

    def _update_color(self, new_detections: List[RobotDetection],
                      tracks: Dict[int, dict], is_red: bool) -> Dict[int, dict]:
        """
        Core per-color tracking logic. Called separately for red and blue.

        Steps:
            1. Match new detections to existing tracks by distance.
            2. Update matched tracks (reset missing_count).
            3. Increment missing_count for unmatched tracks; expire if too old.
            4. Assign new IDs to unmatched new detections.

        Returns the updated tracks dict for this color.
        """
        matches, used_detections = self._match_detections(new_detections, tracks)
        new_tracks = {}

        # --- Step 2: Update matched tracks ---
        for track_id, detection in matches.items():
            detection.track_id = track_id
            new_tracks[track_id] = {'detection': detection, 'missing_count': 0}

        # --- Step 3: Handle unmatched existing tracks ---
        for track_id, track_data in tracks.items():
            if track_id not in matches:
                track_data['missing_count'] += 1
                if track_data['missing_count'] <= self.max_missing_frames:
                    # Keep the track alive - robot might reappear soon
                    new_tracks[track_id] = track_data
                else:
                    # Robot has been gone too long - free its ID for reuse
                    self._free_id(track_id, is_red)

        # --- Step 4: New detections that didn't match any existing track ---
        for i, detection in enumerate(new_detections):
            if i not in used_detections:
                new_id = self._get_next_id(is_red)
                detection.track_id = new_id
                # Remove from free pool in case _get_next_id returned a recycled one
                if is_red:
                    self.red_free_ids.discard(new_id)
                else:
                    self.blue_free_ids.discard(new_id)
                new_tracks[new_id] = {'detection': detection, 'missing_count': 0}

        return new_tracks

    def update(self, detections: List[RobotDetection]) -> List[RobotDetection]:
        """
        Main entry point. Called by BumperDetector.confirm_robots() every frame.
        Mutates each RobotDetection in the list by setting its track_id,
        then returns the same list (modified in place).
        """
        red_detections = [d for d in detections if d.is_red]
        blue_detections = [d for d in detections if d.is_blue]

        self.red_tracks = self._update_color(red_detections, self.red_tracks, is_red=True)
        self.blue_tracks = self._update_color(blue_detections, self.blue_tracks, is_red=False)

        return detections

    def reset(self):
        """
        Wipe all tracking state. Called by /api/tracking/reset in app.py.
        Next frame, all detections get fresh IDs starting from 1 again.
        """
        self.red_tracks.clear()
        self.blue_tracks.clear()
        self.red_free_ids.clear()
        self.blue_free_ids.clear()
        self.red_next_id = 1
        self.blue_next_id = 1

    def get_active_ids(self) -> dict:
        """
        Returns currently tracked IDs for each color.
        Added to /api/detections response in app.py so clients can see which
        IDs are currently live without parsing the full robots list.
        """
        return {
            'red': list(self.red_tracks.keys()),
            'blue': list(self.blue_tracks.keys())
        }
