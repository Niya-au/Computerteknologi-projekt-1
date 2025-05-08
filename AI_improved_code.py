def detect_obstacle(self):
    # More consistent front detection
    # Create continuous angle ranges rather than split front detection
    obstacle_distance_left = min(self.scan_ranges[-90:-60])
    obstacle_distance_front_left = min(self.scan_ranges[-60:-20])
    
    # Better front detection with continuous range
    front_scan = self.scan_ranges[-20:] + self.scan_ranges[:20]
    obstacle_distance_front = min(front_scan)
    
    obstacle_distance_front_right = min(self.scan_ranges[20:60])
    obstacle_distance_right = min(self.scan_ranges[60:90])
    
    # Calculate wider ranges for decision making
    wide_right = (obstacle_distance_right + obstacle_distance_front_right) / 2
    wide_left = (obstacle_distance_left + obstacle_distance_front_left) / 2
    
    passage_width = obstacle_distance_left + obstacle_distance_right
    off_center = obstacle_distance_left - obstacle_distance_right
    absolute_off_center = abs(off_center)
    
    # Store historical data for more stable decision making
    if not hasattr(self, 'recent_fronts'):
        self.recent_fronts = [obstacle_distance_front] * 5
    else:
        self.recent_fronts.pop(0)
        self.recent_fronts.append(obstacle_distance_front)
    
    # More stable front detection using average
    avg_front_distance = sum(self.recent_fronts) / len(self.recent_fronts)
    
    twist = Twist()
    
    # Use proportional control instead of binary thresholds
    # This creates smoother navigation
    
    # Front obstacle avoidance with improved logic
    if avg_front_distance < 0.34:
        print('obstacle in front, distance =', avg_front_distance)
        
        # Only reverse if very close to obstacle
        if avg_front_distance < 0.20:
            reverse_twist = Twist()
            reverse_twist.linear.x = -0.1  # reverse speed
            reverse_twist.angular.z = 0.0
            self.cmd_vel_pub.publish(reverse_twist)
            time.sleep(0.3)  # Shorter reverse time for more responsive behavior
        
        # Intelligent direction choice based on wider space detection
        if wide_right > wide_left + 0.15:  # Add more bias to one side to prevent oscillation
            print('front: choosing right side')
            turn_speed = self.base_angular_speed * min(1.0, (0.34 - avg_front_distance) * 4)
            twist.linear.x = 0.05
            twist.angular.z = -turn_speed  # Turn right proportionally to obstacle proximity
        else:
            print('front: choosing left side')
            turn_speed = self.base_angular_speed * min(1.0, (0.34 - avg_front_distance) * 4)
            twist.linear.x = 0.05
            twist.angular.z = turn_speed  # Turn left proportionally to obstacle proximity
    
    # Side obstacle avoidance with proportional control
    elif obstacle_distance_front_left < 0.30:
        avoid_strength = min(1.0, (0.30 - obstacle_distance_front_left) * 5)
        print(f'obstacle in front left: {obstacle_distance_front_left}, strength: {avoid_strength}')
        twist.angular.z = self.base_angular_speed * avoid_strength * 0.8
        twist.linear.x = self.base_linear_speed * (1 - avoid_strength * 0.8)
    
    elif obstacle_distance_left < 0.20:
        avoid_strength = min(1.0, (0.20 - obstacle_distance_left) * 8)
        print(f'obstacle on left: {obstacle_distance_left}, strength: {avoid_strength}')
        twist.angular.z = self.base_angular_speed * avoid_strength * 0.5
        twist.linear.x = self.base_linear_speed * (1 - avoid_strength * 0.3)
    
    elif obstacle_distance_front_right < 0.30:
        avoid_strength = min(1.0, (0.30 - obstacle_distance_front_right) * 5)
        print(f'obstacle in front right: {obstacle_distance_front_right}, strength: {avoid_strength}')
        twist.angular.z = -self.base_angular_speed * avoid_strength * 0.8
        twist.linear.x = self.base_linear_speed * (1 - avoid_strength * 0.8)
    
    elif obstacle_distance_right < 0.20:
        avoid_strength = min(1.0, (0.20 - obstacle_distance_right) * 8)
        print(f'obstacle on right: {obstacle_distance_right}, strength: {avoid_strength}')
        twist.angular.z = -self.base_angular_speed * avoid_strength * 0.5
        twist.linear.x = self.base_linear_speed * (1 - avoid_strength * 0.3)
    
    # Narrow passage handling with improved centering
    elif passage_width < 1.2:
        print('navigating narrow passage')
        if absolute_off_center > 0.05:
            # Proportional centering
            correction = off_center * 0.6  # Proportional gain
            twist.linear.x = self.narrow_passage_linear_speed * (1 - min(0.8, absolute_off_center))
            twist.angular.z = correction
        else:
            twist.linear.x = self.base_linear_speed
            twist.angular.z = 0.0
    
    else:
        # Default behavior - use teleop twist or implement goal-based navigation
        twist = self.tele_twist
    
    # Apply momentum - avoid sharp changes in direction
    if hasattr(self, 'last_twist'):
        # Blend current command with previous command for smoother motion
        blend_factor = 0.7  # Higher = more smoothing, lower = more responsive
        twist.linear.x = blend_factor * self.last_twist.linear.x + (1 - blend_factor) * twist.linear.x
        twist.angular.z = blend_factor * self.last_twist.angular.z + (1 - blend_factor) * twist.angular.z
    
    # Store this command for next time
    self.last_twist = Twist()
    self.last_twist.linear.x = twist.linear.x
    self.last_twist.angular.z = twist.angular.z
    
    # Publish action
    self.cmd_vel_pub.publish(twist)
    
    # Update metrics
    self.speed_updates = self.speed_updates + 1
    self.speed_accumulation = self.speed_accumulation + twist.linear.x
    
    # Update collision count
    collision_detected = (obstacle_distance_front < 0.17 or 
                          obstacle_distance_left < 0.20 or 
                          obstacle_distance_front_left < 0.20 or 
                          obstacle_distance_right < 0.20 or 
                          obstacle_distance_front_right < 0.20)
    
    if collision_detected and not self.in_collision:
        self.collision_counter = self.collision_counter + 1
        self.in_collision = True
    elif not collision_detected:
        self.in_collision = False
