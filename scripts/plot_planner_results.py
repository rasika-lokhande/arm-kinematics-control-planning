import pandas as pd
import matplotlib.pyplot as plt
from utils.config import config

df = pd.read_csv(config['PLANNER_VALIDATION_CSV_PATH'])

fig, axes = plt.subplots(2, 2, figsize=(12, 10))

# 1. Success vs start-goal distance (scatter, colored by outcome)
ax = axes[0, 0]
success_df = df[df['success'] == True]
fail_df = df[df['success'] == False]
ax.scatter(success_df['start_goal_distance'], [1]*len(success_df), c='green', label='success', alpha=0.6)
ax.scatter(fail_df['start_goal_distance'], [0]*len(fail_df), c='red', label='failure', alpha=0.6)
ax.set_xlabel('start-goal distance (rad)')
ax.set_yticks([0, 1])
ax.set_yticklabels(['fail', 'success'])
ax.set_title('Success vs. start-goal distance')
ax.legend()

# 2. iter_conv distribution (successful trials only)
ax = axes[0, 1]
ax.hist(success_df['iter_conv'], bins=20, color='steelblue', edgecolor='black')
ax.set_xlabel('iterations to converge')
ax.set_ylabel('count')
ax.set_title(f'Convergence iterations (n={len(success_df)})')

# 3. path_cost vs start_goal_distance
ax = axes[1, 0]
ax.scatter(success_df['start_goal_distance'], success_df['path_cost'], alpha=0.6)
ax.plot([0, success_df['start_goal_distance'].max()], [0, success_df['start_goal_distance'].max()],
        'r--', label='y = x (straight-line lower bound)')
ax.set_xlabel('start-goal distance (rad)')
ax.set_ylabel('path cost')
ax.set_title('Path cost vs. distance')
ax.legend()

# 4. tree_size vs start_goal_distance
ax = axes[1, 1]
ax.scatter(df['start_goal_distance'], df['tree_size'], c=df['success'].map({True: 'green', False: 'red'}), alpha=0.6)
ax.set_xlabel('start-goal distance (rad)')
ax.set_ylabel('tree size (nodes)')
ax.set_title('Search effort vs. distance')

plt.tight_layout()
plt.savefig(config['PLANNER_VALIDATION_PLOT_PATH'], dpi=150)
plt.show()

print(df.groupby('success')['start_goal_distance'].describe())