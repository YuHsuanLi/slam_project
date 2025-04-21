import pandas as pd
import numpy as np

def interpolate_to_target_length(df, target_length):
    """
    df: 原始DataFrame
    target_length: 想要的筆數（例如4200）
    """
    # 建立新的索引（均勻分布在原來index範圍）
    original_idx = np.linspace(0, 1, len(df))
    target_idx = np.linspace(0, 1, target_length)

    interpolated = pd.DataFrame()

    for col in df.columns:
        # 只有數值欄位做內插，文字欄位（例如frame_id）直接取最近的
        if np.issubdtype(df[col].dtype, np.number):
            interpolated[col] = np.interp(target_idx, original_idx, df[col])
        else:
            # 文字類型直接用最近的
            interpolated[col] = df[col].iloc[np.searchsorted(original_idx, target_idx, side="right") - 1].values

    return interpolated

def main():
    input_path = '/Users/yuhsuanli/Desktop/projects/slam_project/evaluation/kitti00_odometry_lidar.csv'  # TODO: 換成你的csv路徑
    output_path = '/Users/yuhsuanli/Desktop/projects/slam_project/evaluation/kitti00_odometry_lidar_interp.csv'

    # 讀取csv
    df = pd.read_csv(input_path)

    print(f"原始資料筆數：{len(df)}")
    
    # 內插到4200筆
    interpolated_df = interpolate_to_target_length(df, target_length=4540)

    print(f"內插後資料筆數：{len(interpolated_df)}")

    # 儲存
    interpolated_df.to_csv(output_path, index=False)
    print(f"內插資料已儲存到 {output_path}")

if __name__ == "__main__":
    main()
