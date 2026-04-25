#ifndef INCLINATION_DETECTOR_H
#define INCLINATION_DETECTOR_H
#include <Arduino.h>
#include <queue>
#include <utility>

enum class Inclination : uint8_t
{
    Decline = 0,
    Level = 1,
    Incline = 2,
};

/**
 * @brief 根据支撑相状态与踝关节角度数据，输出当前路面坡度的估计值
 * 
 */
class InclinationDetector
{
    public:
    InclinationDetector();

    /**
     * @brief 设置下坡角度阈值
     * 
     * @param threshold 
     */
    void set_decline_angle(float threshold);
    
    /**
     * @brief 设置上坡角度阈值
     * 
     * @param threshold 
     */
    void set_incline_angle(float threshold);

    /**
     * @brief 检测当前步态与角度数据，判断路面坡度状态
     * 
     * @param is_stance 
     * @param norm_angle 
     * @return Inclination(坡度状态)
     */
    Inclination check(const bool is_stance, const bool fsr_calibrating, const float norm_angle);
    
    private:
    /* Exposed through getters/setters */
    std::pair<float, float> _decline_stance_phase_percent;
    float _incline_angle_theshold;
    float _decline_angle_threshold;

    /* Internal */
    bool _previous_fsr_calibrating;
    int _calibration_step_count;
    const int _steps_until_calibrated = 4;
    bool _previous_should_calibrate;
    bool _returned_this_stance;
    Inclination _prev_estimate;
    bool _prev_stance;
    float _prev_stance_time;
    float _current_stance_start;

    /* Feature Data for Threshold Determination */
    float _incline_features_sum;
    float _decline_features_sum;
    float _calibrated_stance_entry_angle;
    float _calibrated_stance_exit_angle;

    enum class Edge: int
    {
        None = 0,
        Rising = 1,
        Falling = 2, 
    };

    /**
     * @brief Check the current stance for an edge, and handle edge tracking state
     * 
     * @param is_stance 
     * @return Edge 
     */
    Edge _check_for_edge(const bool is_stance);
    
    /**
     * @brief Update the stance phase estimation and return a current estimate.
     * 
     * @param edge
     * @param is_stance 
     * @return float Zero if in swing, stance phase percent if in stance (0-100)
     */
    float _update_stance_phase(const Edge edge, const bool is_stance);
    
    /**
     * @brief Updates the internal thresholds based on angle data at specific time during the gait event
     * 
     */
    void _calculate_gait_features();
    
    /**
     * @brief Store gait feature data for calculation in _calculate_gait_features()
     * 
     * @param edge 
     * @param norm_angle 
     */
    void _update_gait_features(const Edge edge, const float norm_angle);
    
    /**
     * @brief Assuming that there is a rising edge, check if the angle is lower 
     * than the threshold
     * 
     * @param norm_angle 
     * @return true Incline
     * @return false Not Incline
     */
    bool _incline_check(const float norm_angle);
    
    /**
     * @brief Assuming that the stance phase percentage is correct, check if the range 
     * lower than the threshold
     * 
     * @param norm_angle 
     * @return true Decline
     * @return false Not Decline
     */
    bool _decline_check(const float norm_angle);
    
    /**
     * @brief If the Inclination detector should calibrate
     * 
     * @param fsr_calibrating 
     * @param edge 
     * @return true 
     * @return false 
     */
    bool _should_calibrate(const bool fsr_calibrating, const InclinationDetector::Edge edge);
};

#endif