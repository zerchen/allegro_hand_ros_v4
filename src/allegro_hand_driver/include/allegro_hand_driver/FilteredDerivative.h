/// Basic implementation of https://fr.mathworks.com/help/sps/ref/filteredderivativediscreteorcontinuous.html

# pragma once

template<typename TimeType, typename ValueType>
class FilteredDerivative {
  public:
    FilteredDerivative() = default;
    virtual ~FilteredDerivative() = default;

    // Default time constant is zero. Adding new point will raise exception. User needs to set it before.
    void set_time_constant(const TimeType constant);
    void new_point(const TimeType timestamp, const ValueType v);

    ValueType get_raw_value() const {return _raw_value; }
    ValueType get_value_filtered() const {return _value_filtered; }
    ValueType get_raw_derivative() const {return _raw_derivative; }
    ValueType get_derivative_filtered() const {return _derivative_filtered; }

  private:
    TimeType  _time_constant       = 0;
    TimeType  _timestamp           = 0;
    ValueType _internal_state      = 0;
    ValueType _raw_value           = 0;
    ValueType _value_filtered      = 0;
    ValueType _raw_derivative      = 0;
    ValueType _derivative_filtered = 0;
};

template<typename TimeType, typename ValueType>
void FilteredDerivative<TimeType, ValueType>::new_point(const TimeType timestamp, const ValueType v)
{
    const TimeType __sample_time = timestamp - _timestamp;
    const ValueType __alpha = (ValueType) __sample_time / _time_constant;

    // Prevent divinding by zero;
    assert(_time_constant);
    assert(__sample_time);

    // Update internal state
    _internal_state = (1. - __alpha) * _internal_state + __alpha * v;

    // Update user values
    _derivative_filtered = (v - _internal_state) / _time_constant;
    _raw_derivative = (v - _raw_value) / __sample_time;
    _value_filtered = _internal_state;
    _raw_value = v;
    _timestamp = timestamp;
}

template<typename TimeType, typename ValueType>
void FilteredDerivative<TimeType, ValueType>::set_time_constant(const TimeType constant)
{
    _time_constant = constant;
};