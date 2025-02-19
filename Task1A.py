import sympy as sp
import numpy as np 
import control as ctrl

########## Initialization of  variables & provided  
# Define the symbolic variables
x1, x2, u = sp.symbols('x1 x2 u')

# Define the differential equations
x1_dot = -x1 + 2*x1**3 + x2 + 4*u
x2_dot = -x1 - x2 + 2*u
##################################################


def find_equilibrium_points():
    '''
    1. Substitute input(u) = 0 in both equation for finding equilibrium points 
    2. Equate x1_dot, x2_dot equal to zero for finding equilibrium points 
    3. solve the x1_dot, x2_dot equations for the unknown variables and save the value to the variable namely "equi_points"
    '''

    ###### WRITE YOUR CODE HERE ################
    x1_dot_1 = x1_dot.subs(u, 0)
    x2_dot_2 = x2_dot.subs(u, 0)
    equi_points = sp.solve([x1_dot_1, x2_dot_2], (x1, x2))
    ############################################

    return equi_points

def find_A_B_matrices( eq_points):
    '''
    1. Substitute every equilibrium points that you have already find in the find_equilibrium_points() function 
    2. After substituting the equilibrium points, Save the Jacobian matrices A and B as A_matrices, B_matrices  
    '''
    A_matrix = sp.Matrix([
        [sp.diff(x1_dot, x1), sp.diff(x1_dot, x2)],
        [sp.diff(x2_dot, x1), sp.diff(x2_dot, x2)]
    ])
    
    B_matrix = sp.Matrix([
        [sp.diff(x1_dot, u)],
        [sp.diff(x2_dot, u)]
    ])
    A_matrices, B_matrices = [], []
    
    ###### WRITE YOUR CODE HERE ################
    for point in eq_points:
        value = {x1: point[0], x2: point[1]}
        A_matrices.append(A_matrix.subs(value))
        B_matrices.append(B_matrix.subs(value))
    ############################################
    
    return A_matrices,B_matrices

def find_eigen_values(A_matrices):
    '''
    1.  Find the eigen values of all A_matrices (You can use the eigenvals() function of sympy) 
        and append it to the 'eigen_values' list
    2.  With the eigen values, determine whether the system is stable or not and
        append the string 'Stable' if system is stable, else append the string 'Unstable'
        to the 'stability' list 
    '''
    
    eigen_values = []
    stability = []

    ###### WRITE YOUR CODE HERE ################

    for A in A_matrices:
        eig_vals = A.eigenvals()
        eigen_values.append(eig_vals)

        # Check the real part of each eigenvalue to determine stability
        if all(eig_val.as_real_imag()[0] <= 0 for eig_val in eig_vals.keys()):
            stability.append("Stable")
        else:
            stability.append("Unstable")

    ############################################
    return eigen_values, stability

def compute_lqr_gain(jacobians_A, jacobians_B):
    K = 0
    '''
    This function is use to compute the LQR gain matrix K
    1. Use the Jacobian A and B matrix at the equilibrium point (-1,1) and assign it to A and B respectively for computing LQR gain
    2.  Compute the LQR gain of the given system equation (You can use lqr() of control module) 
    3. Take the A matrix corresponding to the Unstable Equilibrium point (-1,1) that you have already found for computing LQR gain.
    4. Assign the value of gain to the variable K
    '''

    # Define the Q and R matrices
    Q = np.eye(2)  # State weighting matrix
    R = np.array([1])  # Control weighting matrix

    ###### WRITE YOUR CODE HERE ################
    eql_points = find_equilibrium_points()
    # Find the index of the target equilibrium point
    target_eq_point = (-1,1)
    target_index = eql_points.index(target_eq_point)

    # Extract the corresponding Jacobian matrices A and B at that equilibrium point
    A = np.array(jacobians_A[target_index]).astype(np.float64)
    B = np.array(jacobians_B[target_index]).astype(np.float64)

    # Calculate the LQR gain matrix K
    K, _, _ = ctrl.lqr(A, B, Q, R)
    ############################################
    return K

def main_function(): # Don't change anything in this function 
    # Find equilibrium points
    eq_points = find_equilibrium_points()
    
    if not eq_points:
        print("No equilibrium points found.")
        return None, None, None, None, None, None
    
    # Find Jacobian matrices
    jacobians_A, jacobians_B = find_A_B_matrices(eq_points)
    
    # For finding eigenvalues and stability of the given equation
    eigen_values, stability = find_eigen_values(jacobians_A)
    
    
    # Compute the LQR gain matrix K
    K = compute_lqr_gain(jacobians_A, jacobians_B)
    
    return eq_points, jacobians_A,  eigen_values, stability, K


def task1a_output():
    '''
    This function will print the results that you have obtained 
    '''
    print("Equilibrium Points:")
    print(eq_points)
    # for i, point in enumerate(eq_points):
    #     print(f"  Point {i + 1}: x1 = {point[0]}, x2 = {point[1]}")
    
    print("\nJacobian Matrices at Equilibrium Points:")
    for i, matrix in enumerate(jacobians_A):
        print(f"  At Point {i + 1}:")
        print(sp.pretty(matrix, use_unicode=True))
    
    print("\nEigenvalues at Equilibrium Points:")
    for i, eigvals_dict in enumerate(eigen_values):
        eigvals_str = ', '.join([f"Eigenvalue: {eigval}: {multiplicity}" for eigval, multiplicity in eigvals_dict.items()])
        print(f"  At Point {i + 1}: {eigvals_str}")
    
    print("\nStability of Equilibrium Points:")
    for i, status in enumerate(stability):
        print(f"  At Point {i + 1}: {status}")
    
    print("\nLQR Gain Matrix K at the selected Equilibrium Point:")
    print(K)


if __name__ == "__main__":
    # Run the main function
    results = main_function()
    
    # This will get the equilibrium points, A_matrix, eigen values, stability of the system
    eq_points, jacobians_A, eigen_values, stability, K = results

    # print the results
    task1a_output()