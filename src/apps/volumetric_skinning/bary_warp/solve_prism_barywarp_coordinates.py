##
#   Python Script to compute generalized barycentric coordinates for prisms.
#   We use scipy's SLSQP to solve for coordinates via a constrained optimization problem.
#   Costly, but only needs to be run if you want to change the skeleton/muscle in the template.
##



import numpy as np
from scipy.optimize import minimize
from tqdm import tqdm

if __name__ == "__main__":
    for gender in ["male", "female"]:
        path = "../../../../data/bary_warp"

        template_positions = np.load(f"{path}/{gender}_template_positions.npy")
        muscle_positions = np.load(f"{path}/{gender}_muscle_positions.npy")
        prism_indices = np.load(f"{path}/{gender}_prism_indices.npy")
        muscle_indices = np.load(f"{path}/{gender}_muscle_indices.npy")

        print(muscle_positions)

        def solve_prism(X, prism):
            A1 = prism[0, :]
            A2 = prism[1, :]
            A3 = prism[2, :]
            B1 = prism[3, :]
            B2 = prism[4, :]
            B3 = prism[5, :]
            x0 = np.array([1/3, 1/3, 1/3, 0.5])

            def pack_vars(x):
                alpha, beta, gamma, xi = x
                A_mix = alpha * A1 + beta * A2 + gamma * A3
                B_mix = alpha * B1 + beta * B2 + gamma * B3
                return xi * A_mix + (1 - xi) * B_mix

            def obj(x):
                comb = pack_vars(x)
                return np.sum((comb - X) ** 2)

            cons = [
                {'type': 'eq', 'fun': lambda x: x[0] + x[1] + x[2] - 1},
                {'type': 'ineq', 'fun': lambda x: x},
                {'type': 'ineq', 'fun': lambda x: 1 - x[3]}
            ]

            res = minimize(
                obj, x0,
                method='SLSQP',
                bounds=[(0, 1)] * 4,
                constraints=cons,
                options={
                    'ftol': 1e-14,
                    'eps': 1e-12,
                    'maxiter': 10000,
                }
            )
            alpha, beta, gamma, xi = res.x

            return np.array([
                xi * alpha, xi * beta, xi * gamma,
                (1 - xi) * alpha, (1 - xi) * beta, (1 - xi) * gamma
            ]), res.fun < 1e-3

        n = muscle_indices.shape[0]
        coefficients = np.zeros((n, 6))

        for i in tqdm(range(n), desc=f"Processing {gender}"):
            muscle_index = int(muscle_indices[i])
            muscle_position = muscle_positions[muscle_index, :]
            prism = prism_indices[i, :].astype(int)
            prism_pos = template_positions[prism, :]

            coeffs, success = solve_prism(muscle_position, prism_pos)
            if not success:
                print(f"[WARNING] Failed to solve for index {i} (muscle idx {muscle_index})")
            coefficients[i, :] = coeffs

        np.save(f"{path}/{gender}_coefficients.npy", coefficients)
