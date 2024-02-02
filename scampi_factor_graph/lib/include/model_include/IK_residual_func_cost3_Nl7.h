// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/rot3.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: IK_residual_func_cost3_Nl7
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     DeltaRot: Rot3
 *     position_vector: Matrix31
 *     Rot_init: Rot3
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3Nl7(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 280

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (103)
  const Scalar _tmp0 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp1 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp2 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp3 = 2 * _tmp2;
  const Scalar _tmp4 = _tmp1 * _tmp3;
  const Scalar _tmp5 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp6 = -_DeltaRot[0] * _Rot_init[0] - _DeltaRot[1] * _Rot_init[1] -
                       _DeltaRot[2] * _Rot_init[2] + _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp7 = 2 * _tmp5 * _tmp6;
  const Scalar _tmp8 = Scalar(0.20999999999999999) * _tmp4 + Scalar(0.20999999999999999) * _tmp7;
  const Scalar _tmp9 = -2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp10 = -2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp11 = -Scalar(0.010999999999999999) * _tmp10 -
                        Scalar(0.010999999999999999) * _tmp9 + Scalar(-0.010999999999999999);
  const Scalar _tmp12 = _tmp3 * _tmp5;
  const Scalar _tmp13 = 2 * _tmp1;
  const Scalar _tmp14 = _tmp13 * _tmp6;
  const Scalar _tmp15 = Scalar(0.20999999999999999) * _tmp12 - Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp16 = _tmp11 - _tmp15;
  const Scalar _tmp17 = _tmp16 + _tmp8;
  const Scalar _tmp18 = _tmp13 * _tmp5;
  const Scalar _tmp19 = _tmp3 * _tmp6;
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp21 = 1 - 2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp23 =
      -Scalar(0.010999999999999999) * _tmp12 - Scalar(0.010999999999999999) * _tmp14;
  const Scalar _tmp24 = -_tmp22 + _tmp23;
  const Scalar _tmp25 = _tmp20 + _tmp24;
  const Scalar _tmp26 = _tmp25 + position_vector(0, 0) + Scalar(1.79662371);
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp28 =
      -Scalar(0.010999999999999999) * _tmp4 + Scalar(0.010999999999999999) * _tmp7;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp30 = _tmp28 - _tmp29;
  const Scalar _tmp31 = _tmp27 + _tmp30;
  const Scalar _tmp32 = _tmp31 + position_vector(1, 0) + Scalar(-4.8333311099999996);
  const Scalar _tmp33 = std::pow(Scalar(std::pow(_tmp26, Scalar(2)) + std::pow(_tmp32, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp34 = _tmp26 * _tmp33;
  const Scalar _tmp35 = _tmp16 - _tmp8;
  const Scalar _tmp36 = _tmp34 * _tmp35;
  const Scalar _tmp37 = -_tmp17 * _tmp34 + _tmp36;
  const Scalar _tmp38 = _tmp32 * _tmp33;
  const Scalar _tmp39 = -_tmp27;
  const Scalar _tmp40 = _tmp30 + _tmp39;
  const Scalar _tmp41 = _tmp40 + position_vector(1, 0) + Scalar(8.3196563700000006);
  const Scalar _tmp42 = -_tmp20;
  const Scalar _tmp43 = _tmp24 + _tmp42;
  const Scalar _tmp44 = _tmp43 + position_vector(0, 0) + Scalar(1.9874742000000001);
  const Scalar _tmp45 = Scalar(1.0) / (_tmp44);
  const Scalar _tmp46 = _tmp41 * _tmp45;
  const Scalar _tmp47 = Scalar(1.0) / (_tmp34 * _tmp46 - _tmp38);
  const Scalar _tmp48 = _tmp28 + _tmp29;
  const Scalar _tmp49 = _tmp27 + _tmp48;
  const Scalar _tmp50 = _tmp49 + position_vector(1, 0) + Scalar(-4.7752063900000001);
  const Scalar _tmp51 = _tmp22 + _tmp23;
  const Scalar _tmp52 = _tmp20 + _tmp51;
  const Scalar _tmp53 = _tmp52 + position_vector(0, 0) + Scalar(-2.71799795);
  const Scalar _tmp54 = std::pow(Scalar(std::pow(_tmp50, Scalar(2)) + std::pow(_tmp53, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp55 = _tmp50 * _tmp54;
  const Scalar _tmp56 = _tmp53 * _tmp54;
  const Scalar _tmp57 = _tmp46 * _tmp56 - _tmp55;
  const Scalar _tmp58 = _tmp47 * _tmp57;
  const Scalar _tmp59 = _tmp11 + _tmp15 + _tmp8;
  const Scalar _tmp60 = _tmp35 * _tmp56;
  const Scalar _tmp61 = _tmp17 * _tmp38 - _tmp36 * _tmp46;
  const Scalar _tmp62 = Scalar(1.0) * _tmp40;
  const Scalar _tmp63 = Scalar(1.0) * _tmp43;
  const Scalar _tmp64 = (-_tmp25 + _tmp63) / (_tmp31 - _tmp62);
  const Scalar _tmp65 = -_tmp37 * _tmp58 - _tmp56 * _tmp59 + _tmp60 -
                        _tmp64 * (-_tmp46 * _tmp60 + _tmp55 * _tmp59 - _tmp58 * _tmp61);
  const Scalar _tmp66 = Scalar(1.0) / (_tmp65);
  const Scalar _tmp67 = _tmp62 * _tmp64 + _tmp63;
  const Scalar _tmp68 = 0;
  const Scalar _tmp69 = _tmp34 * _tmp58;
  const Scalar _tmp70 =
      std::sqrt(Scalar(std::pow(_tmp41, Scalar(2)) + std::pow(_tmp44, Scalar(2))));
  const Scalar _tmp71 = _tmp45 * _tmp70;
  const Scalar _tmp72 = Scalar(1.0) / (_tmp70);
  const Scalar _tmp73 = _tmp71 * (-_tmp40 * _tmp44 * _tmp72 + _tmp41 * _tmp43 * _tmp72);
  const Scalar _tmp74 = -_tmp25 * _tmp38 + _tmp31 * _tmp34 + _tmp34 * _tmp73;
  const Scalar _tmp75 = _tmp49 * _tmp56 - _tmp52 * _tmp55 + _tmp56 * _tmp73 - _tmp58 * _tmp74;
  const Scalar _tmp76 = Scalar(1.0) / (_tmp75);
  const Scalar _tmp77 = Scalar(1.0) * _tmp76;
  const Scalar _tmp78 = _tmp42 + _tmp51;
  const Scalar _tmp79 = _tmp39 + _tmp48;
  const Scalar _tmp80 = _tmp79 + position_vector(1, 0) + Scalar(8.3888750099999996);
  const Scalar _tmp81 = _tmp78 + position_vector(0, 0) + Scalar(-2.5202214700000001);
  const Scalar _tmp82 = std::pow(Scalar(std::pow(_tmp80, Scalar(2)) + std::pow(_tmp81, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp83 = _tmp80 * _tmp82;
  const Scalar _tmp84 = _tmp81 * _tmp82;
  const Scalar _tmp85 = fh1 * (_tmp78 * _tmp83 - _tmp79 * _tmp84);
  const Scalar _tmp86 = _tmp46 * _tmp47;
  const Scalar _tmp87 = -_tmp35 + _tmp37 * _tmp86 - _tmp64 * (_tmp35 * _tmp46 + _tmp61 * _tmp86);
  const Scalar _tmp88 = _tmp66 * _tmp75;
  const Scalar _tmp89 = _tmp65 * _tmp76;
  const Scalar _tmp90 = _tmp87 + _tmp89 * (-_tmp73 + _tmp74 * _tmp86 - _tmp87 * _tmp88);
  const Scalar _tmp91 = _tmp56 * _tmp66;
  const Scalar _tmp92 = _tmp57 * _tmp66;
  const Scalar _tmp93 = -_tmp46 - _tmp90 * _tmp92;
  const Scalar _tmp94 = _tmp34 * _tmp47;
  const Scalar _tmp95 = _tmp84 * fh1;
  const Scalar _tmp96 = Scalar(1.0) * _tmp47;
  const Scalar _tmp97 = -_tmp37 * _tmp96 + _tmp61 * _tmp64 * _tmp96;
  const Scalar _tmp98 = _tmp89 * (-_tmp74 * _tmp96 - _tmp88 * _tmp97) + _tmp97;
  const Scalar _tmp99 = -_tmp92 * _tmp98 + Scalar(1.0);
  const Scalar _tmp100 = _tmp83 * fh1;
  const Scalar _tmp101 = _tmp77 * _tmp85;
  const Scalar _tmp102 = _tmp0 * _tmp68;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = std::exp(-fh1);
  _res(1, 0) = std::exp(_tmp0 * _tmp71 * (_tmp56 * _tmp68 - _tmp68 * _tmp69) +
                        _tmp100 * _tmp71 * (_tmp91 * _tmp98 + _tmp94 * _tmp99) +
                        _tmp71 * _tmp85 * (_tmp56 * _tmp77 - _tmp69 * _tmp77) +
                        _tmp71 * _tmp95 * (_tmp90 * _tmp91 + _tmp93 * _tmp94 + Scalar(1.0)));
  _res(2, 0) = std::exp(-_tmp100 * _tmp47 * _tmp99 + _tmp101 * _tmp58 + _tmp102 * _tmp58 -
                        _tmp47 * _tmp93 * _tmp95);
  _res(3, 0) = std::exp(-_tmp100 * _tmp66 * _tmp98 - _tmp101 - _tmp102 - _tmp66 * _tmp90 * _tmp95);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
