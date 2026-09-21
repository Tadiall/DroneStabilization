"""
Correcteur PID générique, avec saturation de sortie et anti windup simple
(l'intégrale n'est mise à jour que si la commande n'est pas saturée).
"""


class PID:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0

        # Calcul provisoire sans l'intégrale mise à jour, pour tester la saturation
        provisional_output = (
            self.kp * error + self.ki * self._integral + self.kd * derivative
        )

        # Anti windup : on n'accumule l'intégrale que si on n'est pas saturé
        if -self.output_limit < provisional_output < self.output_limit:
            self._integral += error * dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = max(-self.output_limit, min(self.output_limit, output))

        self._prev_error = error
        return output
