"""
Modèle physique : rotation autour d'un seul axe (tangage).

I * theta_ddot = tau - damping * theta_dot
"""


class RotationalAxis:
    def __init__(self, inertia, damping=0.02, theta0=0.0, theta_dot0=0.0):
        self.I = inertia
        self.damping = damping
        self.theta = theta0        # rad
        self.theta_dot = theta_dot0  # rad/s

    def step(self, torque, dt):
        """Intègre un pas de temps par méthode d'Euler explicite."""
        theta_ddot = (torque - self.damping * self.theta_dot) / self.I
        self.theta_dot += theta_ddot * dt
        self.theta += self.theta_dot * dt
        return self.theta, self.theta_dot
