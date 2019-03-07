// Given a time between 0 and 1, evaluates a cubic polynomial with
// the given endpoint and tangent values at the beginning (0) and
// end (1) of the interval.  Optionally, one can request a derivative
// of the spline (0=no derivative, 1=first derivative, 2=2nd derivative).
template <class T>
inline T Spline<T>::cubicSplineUnitInterval(
    const T& position0, const T& position1, const T& tangent0,
    const T& tangent1, double normalizedTime, int derivative) {
  // TODO (Animation) Task 1a
  double t, t2, t3, h00, h10, h01, h11;
  t = normalizedTime;
  t2 = t*t;
  t3 = t2*t;
  switch(derivative)
  {
    case 1: //1st derivative
      h00 = 6.*t2 - 6.*t;
      h10 = 3.*t2 - 4.*t + 1.;
      h01 = -6.*t2 + 6.*t;
      h11 = 3.*t2 - 2.*t;
      break;
    case 2: //2nd derivative
      h00 = 12.*t - 6.;
      h10 = 6.*t - 4.;
      h01 = -12.*t + 6.;
      h11 = 6.*t - 2.;
      break;
    default: //no derivative
      h00 = 2.*t3 - 3.*t2 +1.;
      h10 = t3 - 2.*t2 + t;
      h01 = -2.*t3 + 3.*t2;
      h11 = t3 - t2;
      break;
  }
  return T(h00*position0 + h10*tangent0 + h01*position1 + h11*tangent1);
}

// Returns a state interpolated between the values directly before and after the
// given time.
template <class T>
inline T Spline<T>::evaluate(double time, int derivative) {
  // TODO (Animation) Task 1b
  int ksize = knots.size();
  if (ksize < 1)
    return T();
  //// Case 2: Only one knot, return knot unless derivative is specified.  In that case return T() ////
  else if (ksize == 1)
  {
    if (derivative)
      return T(); // Case 2a: One knot, return T() for either derivative //
    else
      return knots.begin()->second; // Case 2b: One knot, return it //
  }
  //// Case 3: Time is less than or equal to beginning knot ////
  else if (time <= knots.begin()->first)
  {
    if (derivative)
      return T();
    else
      return knots.begin()->second;
  }
  //// Case 4: Time is greater than or equal to end knot ////
  else if (time >= knots.rbegin()->first)
  {
    if (derivative)
      return T();
    else
      return knots.rbegin()->second;
  }
  //// Have at least two knots at this point ////
  else
  {
    T p0, p1, p2, p3, m1, m2, result;
    double t0, t1, t2, t3, tdiff, tnorm;
    KnotIter it;
    // Get p2 and p1 //
    it = knots.upper_bound(time);
    p2 = it->second;
    t2 = it->first;
    --it;
    p1 = it->second;
    t1 = it->first;
    
    // Get p0-> if begin(), use mirror technique //
    if (it == knots.begin())
    {
      p0 = p1 - (p2 - p1);
      t0 = t1 - (t2 - t1);
    }
    else{
      --it;
      p0 = it->second;
      t0 = it->first;
      ++it;
    }
    // Get p3-> if null ptr, use mirror technique //
    ++it;++it;
    if (it == knots.end())
    {
      p3 = p2 + (p2 - p1);
      t3 = t2 + (t2 - t1);
    }
    else{
      p3 = it->second;
      t3 = it->first;
    }
    
    // Interpolate //
    tdiff = t2 - t1;
    tnorm = (time-t1)/tdiff;
    m1 = (p2-p0)/(t2-t0)*tdiff;
    m2 = (p3-p1)/(t3-t1)*tdiff;
    

    result = cubicSplineUnitInterval(p1, p2, m1, m2, tnorm, derivative);
    // Normalize if we took derivatives //
    if(derivative >= 1)
      result/=tdiff;
    if(derivative == 2)
      result/=tdiff;
    return result;
    //return knots.begin()->second;
}
}

// Removes the knot closest to the given time,
//    within the given tolerance..
// returns true iff a knot was removed.
template <class T>
inline bool Spline<T>::removeKnot(double time, double tolerance) {
  // Empty maps have no knots.
  if (knots.size() < 1) {
    return false;
  }

  // Look up the first element > or = to time.
  typename std::map<double, T>::iterator t2_iter = knots.lower_bound(time);
  typename std::map<double, T>::iterator t1_iter;
  t1_iter = t2_iter;
  t1_iter--;

  if (t2_iter == knots.end()) {
    t2_iter = t1_iter;
  }

  // Handle tolerance bounds,
  // because we are working with floating point numbers.
  double t1 = (*t1_iter).first;
  double t2 = (*t2_iter).first;

  double d1 = fabs(t1 - time);
  double d2 = fabs(t2 - time);

  if (d1 < tolerance && d1 < d2) {
    knots.erase(t1_iter);
    return true;
  }

  if (d2 < tolerance && d2 < d1) {
    knots.erase(t2_iter);
    return t2;
  }

  return false;
}

// Sets the value of the spline at a given time (i.e., knot),
// creating a new knot at this time if necessary.
template <class T>
inline void Spline<T>::setValue(double time, T value) {
  knots[time] = value;
}

template <class T>
inline T Spline<T>::operator()(double time) {
  return evaluate(time);
}
