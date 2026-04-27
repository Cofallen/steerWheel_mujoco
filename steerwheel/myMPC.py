import numpy as np
from scipy.optimize import minimize

class MyLQR:
    def __init__(self, dt):
        self.dt = dt
        self.Q = np.diag([10, 10, 10])
        self.R = np.diag([0.1, 0.1, 0.1])
        
    def linearize(self, theta):
        A = np.eye(3) + np.array([
            [0, 0, -np.sin(theta) - np.cos(theta)],
            [0, 0, np.cos(theta) - np.sin(theta)],
            
            [0, 0, 0]
        ]) * self.dt
        
        B = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ]) * self.dt
        
        return A, B
    
    def solve(self, x, x_ref):
        theta = x[2]
        A, B = self.linearize(theta)
        P = self.Q.copy()
        for _ in range(10):
            K = np.linalg.inv(self.R + B.T @ P @ B) @ (B.T @ P @ A)
            P = self.Q + A.T @ P @ (A - B @ K)
            
        u = -K @ (x - x_ref)
        return u
    
# 含约束MPC 调参比较重要
class MyMPC:
    def __init__(self, dt, N=10):
        self.dt = dt
        self.N = N
        self.Q = np.diag([30.0, 30.0, 800.0])     # 状态误差
        self.R = np.diag([0.1, 0.1, 1.0])     # 控制输入
         # ===== 控制约束 =====
        self.u_min = np.array([-1.0, -1.0, -10.0])
        self.u_max = np.array([ 1.0,  1.0,  10.0])
        
    def linearize(self, theta):
        A = np.eye(3) + np.array([
            [0, 0, -np.sin(theta) - np.cos(theta)],
            [0, 0, np.cos(theta) - np.sin(theta)],
            [0, 0, 0]
        ]) * self.dt
        
        B = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ]) * self.dt
        
        return A, B
    
    def build_prediction(self, A, B):
        n = A.shape[0]
        m = B.shape[1]
        
        Phi = np.zeros((n*self.N, n))
        Gamma = np.zeros((n*self.N, m*self.N))
        
        A_power = np.eye(n)
        
        for i in range(self.N):
            Phi[i*n:(i+1)*n] = A_power
            A_power = A @ A_power
        
        for i in range(self.N):
            for j in range(i+1):
                A_power = np.linalg.matrix_power(A, i-j)
                Gamma[i*n:(i+1)*n, j*m:(j+1)*m] = A_power @ B
        return Phi, Gamma
    
    def solve(self, x, x_ref):
        theta = x[2]
        
        A, B = self.linearize(theta)
        m = B.shape[1]
        Phi, Gamma = self.build_prediction(A, B)
        
        Q_bar = np.kron(np.eye(self.N), self.Q)
        R_bar = np.kron(np.eye(self.N), self.R)
        x_ref_bar = np.tile(x_ref, self.N)
        
        H = Gamma.T @ Q_bar @ Gamma + R_bar
        f = Gamma.T @ Q_bar @ (Phi @ x - x_ref_bar)
        
        def cost(U):
             return 0.5 * U.T @ H @ U + f.T @ U
         
        # ===== 控制约束 =====
        bounds = []
        for _ in range(self.N):
            for i in range(m):
                bounds.append((self.u_min[i], self.u_max[i]))

        # 初值
        U0 = np.zeros(m*self.N)

        res = minimize(cost, U0, bounds=bounds, method='SLSQP')

        if not res.success:
            print("⚠️ QP求解失败:", res.message)
            return np.zeros(3)

        U_opt = res.x
        return U_opt[:3]