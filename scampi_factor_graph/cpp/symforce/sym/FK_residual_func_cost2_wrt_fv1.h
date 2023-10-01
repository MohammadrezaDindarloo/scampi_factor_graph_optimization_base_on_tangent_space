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
 * Symbolic function: FK_residual_func_cost2_wrt_fv1
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     DeltaRot: Rot3
 *     position_vector: Matrix31
 *     rot_init_x: Scalar
 *     rot_init_y: Scalar
 *     rot_init_z: Scalar
 *     rot_init_w: Scalar
 *     lc0: Scalar
 *     lc1: Scalar
 *     lc2: Scalar
 *     lc3: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> FkResidualFuncCost2WrtFv1(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w, const Scalar lc0,
    const Scalar lc1, const Scalar lc2, const Scalar lc3, const Scalar epsilon) {
  // Total ops: 597

  // Unused inputs
  (void)lc0;
  (void)lc1;
  (void)lc2;
  (void)lc3;
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();

  // Intermediate terms (192)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
  const Scalar _tmp3 = _DeltaRot[0] * rot_init_w - _DeltaRot[1] * rot_init_z +
                       _DeltaRot[2] * rot_init_y + _DeltaRot[3] * rot_init_x;
  const Scalar _tmp4 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp5 = -_DeltaRot[0] * rot_init_y + _DeltaRot[1] * rot_init_x +
                       _DeltaRot[2] * rot_init_w + _DeltaRot[3] * rot_init_z;
  const Scalar _tmp6 = 1 - 2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp7 = Scalar(0.20999999999999999) * _tmp4 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp8 = -_tmp7;
  const Scalar _tmp9 = _DeltaRot[0] * rot_init_z + _DeltaRot[1] * rot_init_w -
                       _DeltaRot[2] * rot_init_x + _DeltaRot[3] * rot_init_y;
  const Scalar _tmp10 = 2 * _tmp5;
  const Scalar _tmp11 = _tmp10 * _tmp9;
  const Scalar _tmp12 = -2 * _DeltaRot[0] * rot_init_x - 2 * _DeltaRot[1] * rot_init_y -
                        2 * _DeltaRot[2] * rot_init_z + 2 * _DeltaRot[3] * rot_init_w;
  const Scalar _tmp13 = _tmp12 * _tmp3;
  const Scalar _tmp14 = _tmp11 - _tmp13;
  const Scalar _tmp15 = Scalar(0.010999999999999999) * _tmp14;
  const Scalar _tmp16 = -_tmp15;
  const Scalar _tmp17 = 2 * _tmp3 * _tmp9;
  const Scalar _tmp18 = _tmp12 * _tmp5;
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp20 = _tmp16 - _tmp19;
  const Scalar _tmp21 = _tmp20 + _tmp8;
  const Scalar _tmp22 = _tmp21 + position_vector(1, 0);
  const Scalar _tmp23 = Scalar(0.20999999999999999) * _tmp17 - Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp24 = -_tmp23;
  const Scalar _tmp25 = _tmp10 * _tmp3;
  const Scalar _tmp26 = _tmp12 * _tmp9;
  const Scalar _tmp27 = _tmp25 + _tmp26;
  const Scalar _tmp28 = -Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp29 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp31 = _tmp28 - _tmp30;
  const Scalar _tmp32 = _tmp24 + _tmp31;
  const Scalar _tmp33 = _tmp32 + position_vector(0, 0);
  const Scalar _tmp34 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp35 = _tmp28 + _tmp30;
  const Scalar _tmp36 = _tmp23 + _tmp35;
  const Scalar _tmp37 = _tmp36 + position_vector(0, 0);
  const Scalar _tmp38 = _tmp37 + Scalar(-2.71799795);
  const Scalar _tmp39 = _tmp19 + _tmp7;
  const Scalar _tmp40 = _tmp16 + _tmp39;
  const Scalar _tmp41 = _tmp40 + position_vector(1, 0);
  const Scalar _tmp42 = _tmp41 + Scalar(-4.7752063900000001);
  const Scalar _tmp43 = std::pow(Scalar(std::pow(_tmp38, Scalar(2)) + std::pow(_tmp42, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp44 = _tmp38 * _tmp43;
  const Scalar _tmp45 = _tmp24 + _tmp35;
  const Scalar _tmp46 = _tmp45 + position_vector(0, 0);
  const Scalar _tmp47 = _tmp46 + Scalar(-2.5202214700000001);
  const Scalar _tmp48 = _tmp16 + _tmp19 + _tmp8;
  const Scalar _tmp49 = _tmp48 + position_vector(1, 0);
  const Scalar _tmp50 = _tmp49 + Scalar(8.3888750099999996);
  const Scalar _tmp51 =
      std::sqrt(Scalar(std::pow(_tmp47, Scalar(2)) + std::pow(_tmp50, Scalar(2))));
  const Scalar _tmp52 = Scalar(1.0) / (_tmp51);
  const Scalar _tmp53 = Scalar(1.0) / (_tmp47);
  const Scalar _tmp54 = _tmp51 * _tmp53;
  const Scalar _tmp55 = _tmp54 * (_tmp45 * _tmp50 * _tmp52 - _tmp47 * _tmp48 * _tmp52);
  const Scalar _tmp56 = _tmp42 * _tmp43;
  const Scalar _tmp57 = -_tmp36 * _tmp56 + _tmp40 * _tmp44 + _tmp44 * _tmp55;
  const Scalar _tmp58 = _tmp50 * _tmp53;
  const Scalar _tmp59 = Scalar(1.0) / (_tmp44 * _tmp58 - _tmp56);
  const Scalar _tmp60 = Scalar(1.0) * _tmp59;
  const Scalar _tmp61 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp62 = -_tmp61;
  const Scalar _tmp63 = -Scalar(0.010999999999999999) * _tmp29 -
                        Scalar(0.010999999999999999) * _tmp4 + Scalar(-0.010999999999999999);
  const Scalar _tmp64 = Scalar(0.20999999999999999) * _tmp25 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp65 = _tmp63 + _tmp64;
  const Scalar _tmp66 = _tmp62 + _tmp65;
  const Scalar _tmp67 = _tmp58 * _tmp66;
  const Scalar _tmp68 = _tmp61 + _tmp65;
  const Scalar _tmp69 = -_tmp44 * _tmp67 + _tmp56 * _tmp68;
  const Scalar _tmp70 = _tmp60 * _tmp69;
  const Scalar _tmp71 = Scalar(1.0) * _tmp48;
  const Scalar _tmp72 = -_tmp71;
  const Scalar _tmp73 = Scalar(1.0) / (_tmp40 + _tmp72);
  const Scalar _tmp74 = Scalar(1.0) * _tmp45;
  const Scalar _tmp75 = _tmp73 * (-_tmp36 + _tmp74);
  const Scalar _tmp76 = _tmp44 * _tmp66 - _tmp44 * _tmp68;
  const Scalar _tmp77 = -_tmp60 * _tmp76 + _tmp70 * _tmp75;
  const Scalar _tmp78 = _tmp20 + _tmp7;
  const Scalar _tmp79 = _tmp78 + position_vector(1, 0);
  const Scalar _tmp80 = _tmp79 + Scalar(-4.8333311099999996);
  const Scalar _tmp81 = _tmp23 + _tmp31;
  const Scalar _tmp82 = _tmp81 + position_vector(0, 0);
  const Scalar _tmp83 = _tmp82 + Scalar(1.79662371);
  const Scalar _tmp84 = std::pow(Scalar(std::pow(_tmp80, Scalar(2)) + std::pow(_tmp83, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp85 = _tmp83 * _tmp84;
  const Scalar _tmp86 = _tmp80 * _tmp84;
  const Scalar _tmp87 = _tmp58 * _tmp85 - _tmp86;
  const Scalar _tmp88 = _tmp59 * _tmp87;
  const Scalar _tmp89 = _tmp63 - _tmp64;
  const Scalar _tmp90 = _tmp61 + _tmp89;
  const Scalar _tmp91 = _tmp66 * _tmp85;
  const Scalar _tmp92 = -_tmp58 * _tmp91 - _tmp69 * _tmp88 + _tmp86 * _tmp90;
  const Scalar _tmp93 = -_tmp75 * _tmp92 - _tmp76 * _tmp88 - _tmp85 * _tmp90 + _tmp91;
  const Scalar _tmp94 = Scalar(1.0) / (_tmp93);
  const Scalar _tmp95 = _tmp55 * _tmp85 - _tmp57 * _tmp88 + _tmp78 * _tmp85 - _tmp81 * _tmp86;
  const Scalar _tmp96 = _tmp94 * _tmp95;
  const Scalar _tmp97 = Scalar(1.0) / (_tmp95);
  const Scalar _tmp98 = _tmp93 * _tmp97;
  const Scalar _tmp99 = _tmp98 * (-_tmp57 * _tmp60 - _tmp77 * _tmp96);
  const Scalar _tmp100 = _tmp77 + _tmp99;
  const Scalar _tmp101 = _tmp85 * _tmp94;
  const Scalar _tmp102 = _tmp87 * _tmp94;
  const Scalar _tmp103 = -_tmp100 * _tmp102 + Scalar(1.0);
  const Scalar _tmp104 = _tmp44 * _tmp59;
  const Scalar _tmp105 = _tmp22 + Scalar(8.3196563700000006);
  const Scalar _tmp106 = _tmp33 + Scalar(1.9874742000000001);
  const Scalar _tmp107 =
      std::pow(Scalar(std::pow(_tmp105, Scalar(2)) + std::pow(_tmp106, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp108 = _tmp105 * _tmp107;
  const Scalar _tmp109 = _tmp108 * fh1;
  const Scalar _tmp110 = _tmp58 * _tmp59;
  const Scalar _tmp111 = _tmp110 * _tmp69 + _tmp67;
  const Scalar _tmp112 = _tmp110 * _tmp76 - _tmp111 * _tmp75 - _tmp66;
  const Scalar _tmp113 = _tmp98 * (_tmp110 * _tmp57 - _tmp112 * _tmp96 - _tmp55);
  const Scalar _tmp114 = _tmp112 + _tmp113;
  const Scalar _tmp115 = -_tmp102 * _tmp114 - _tmp58;
  const Scalar _tmp116 = _tmp106 * _tmp107;
  const Scalar _tmp117 = _tmp116 * fh1;
  const Scalar _tmp118 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp119 = _tmp71 * _tmp75 + _tmp74;
  const Scalar _tmp120 = 0;
  const Scalar _tmp121 = _tmp120 * _tmp94;
  const Scalar _tmp122 = _tmp121 * _tmp88;
  const Scalar _tmp123 = _tmp54 * (_tmp121 * _tmp85 - _tmp122 * _tmp44);
  const Scalar _tmp124 = Scalar(1.0) * _tmp97;
  const Scalar _tmp125 = _tmp60 * _tmp87 * _tmp97;
  const Scalar _tmp126 = fh1 * (_tmp108 * _tmp32 - _tmp116 * _tmp21);
  const Scalar _tmp127 = -_tmp109 * _tmp54 * (_tmp100 * _tmp101 + _tmp103 * _tmp104) -
                         _tmp117 * _tmp54 * (_tmp101 * _tmp114 + _tmp104 * _tmp115 + Scalar(1.0)) -
                         _tmp118 * _tmp123 -
                         _tmp126 * _tmp54 * (_tmp124 * _tmp85 - _tmp125 * _tmp44);
  const Scalar _tmp128 = Scalar(1.0) / (_tmp127);
  const Scalar _tmp129 = _tmp72 + _tmp78;
  const Scalar _tmp130 = _tmp129 * _tmp75;
  const Scalar _tmp131 = Scalar(1.0) / (-_tmp130 + _tmp74 - _tmp81);
  const Scalar _tmp132 = Scalar(1.0) * _tmp131;
  const Scalar _tmp133 = _tmp132 * _tmp98;
  const Scalar _tmp134 = -_tmp124 * _tmp92 + _tmp129 * _tmp133;
  const Scalar _tmp135 = Scalar(1.0) * _tmp73;
  const Scalar _tmp136 = _tmp119 * _tmp131;
  const Scalar _tmp137 = _tmp92 * _tmp94;
  const Scalar _tmp138 = _tmp73 * (-_tmp120 * _tmp137 - _tmp129 * _tmp136 + _tmp72);
  const Scalar _tmp139 = -Scalar(1.0) * _tmp136 - Scalar(1.0) * _tmp138 + Scalar(1.0);
  const Scalar _tmp140 = _tmp129 * _tmp131;
  const Scalar _tmp141 = -_tmp100 * _tmp137 + _tmp140 * _tmp99 - _tmp70;
  const Scalar _tmp142 = fh1 * (_tmp62 + _tmp89);
  const Scalar _tmp143 = _tmp116 * _tmp142 + Scalar(3.29616) * _tmp27 + _tmp32 * fv1;
  const Scalar _tmp144 = _tmp129 * _tmp73;
  const Scalar _tmp145 = Scalar(1.0) * _tmp132 * _tmp144 - Scalar(1.0) * _tmp132;
  const Scalar _tmp146 = -_tmp108 * _tmp142 - Scalar(3.29616) * _tmp14 - _tmp21 * fv1;
  const Scalar _tmp147 = _tmp73 * (_tmp130 * _tmp132 + Scalar(1.0));
  const Scalar _tmp148 = _tmp132 * _tmp75;
  const Scalar _tmp149 = -Scalar(1.0) * _tmp147 + Scalar(1.0) * _tmp148;
  const Scalar _tmp150 = _tmp111 + _tmp113 * _tmp140 - _tmp114 * _tmp137;
  const Scalar _tmp151 = Scalar(1.0) * _tmp109 * (_tmp132 * _tmp99 - _tmp135 * _tmp141) +
                         Scalar(1.0) * _tmp117 * (_tmp113 * _tmp132 - _tmp135 * _tmp150) +
                         _tmp118 * _tmp139 + Scalar(1.0) * _tmp126 * (_tmp133 - _tmp134 * _tmp135) +
                         _tmp143 * _tmp145 + _tmp146 * _tmp149;
  const Scalar _tmp152 = std::asinh(_tmp128 * _tmp151);
  const Scalar _tmp153 = Scalar(1.0) * _tmp152;
  const Scalar _tmp154 = Scalar(9.6622558468725703) * _tmp127;
  const Scalar _tmp155 =
      -_tmp152 * _tmp154 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp46), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp49 - 1), Scalar(2))));
  const Scalar _tmp156 = Scalar(0.1034955) * _tmp128;
  const Scalar _tmp157 = _tmp155 * _tmp156;
  const Scalar _tmp158 = Scalar(9.6622558468725703) * _tmp123;
  const Scalar _tmp159 = std::pow(_tmp127, Scalar(-2));
  const Scalar _tmp160 = _tmp15 + _tmp39;
  const Scalar _tmp161 = _tmp123 * _tmp159;
  const Scalar _tmp162 =
      (_tmp128 * (-_tmp139 + _tmp145 * _tmp32 + _tmp149 * _tmp160) - _tmp151 * _tmp161) /
      std::sqrt(Scalar(std::pow(_tmp151, Scalar(2)) * _tmp159 + 1));
  const Scalar _tmp163 = _tmp132 * _tmp143;
  const Scalar _tmp164 = _tmp109 * _tmp141 * _tmp73 + _tmp117 * _tmp150 * _tmp73 +
                         _tmp118 * _tmp138 + _tmp126 * _tmp134 * _tmp73 - _tmp144 * _tmp163 +
                         _tmp146 * _tmp147;
  const Scalar _tmp165 = _tmp118 * _tmp121;
  const Scalar _tmp166 = _tmp103 * _tmp109 * _tmp59 + _tmp115 * _tmp117 * _tmp59 -
                         _tmp125 * _tmp126 - _tmp165 * _tmp88;
  const Scalar _tmp167 = Scalar(1.0) / (_tmp166);
  const Scalar _tmp168 = std::asinh(_tmp164 * _tmp167);
  const Scalar _tmp169 = Scalar(1.0) * _tmp168;
  const Scalar _tmp170 = Scalar(9.6622558468725703) * _tmp166;
  const Scalar _tmp171 =
      -_tmp168 * _tmp170 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp37), Scalar(2)) +
                     std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp41), Scalar(2))));
  const Scalar _tmp172 = Scalar(0.1034955) * _tmp167;
  const Scalar _tmp173 = _tmp171 * _tmp172;
  const Scalar _tmp174 = Scalar(9.6622558468725703) * _tmp121;
  const Scalar _tmp175 = _tmp174 * _tmp88;
  const Scalar _tmp176 = std::pow(_tmp166, Scalar(-2));
  const Scalar _tmp177 = _tmp132 * _tmp32;
  const Scalar _tmp178 = _tmp122 * _tmp176;
  const Scalar _tmp179 =
      (-_tmp164 * _tmp178 + _tmp167 * (-_tmp138 - _tmp144 * _tmp177 + _tmp147 * _tmp160)) /
      std::sqrt(Scalar(std::pow(_tmp164, Scalar(2)) * _tmp176 + 1));
  const Scalar _tmp180 =
      _tmp100 * _tmp109 * _tmp94 + _tmp114 * _tmp117 * _tmp94 + _tmp124 * _tmp126 + _tmp165;
  const Scalar _tmp181 = Scalar(1.0) / (_tmp180);
  const Scalar _tmp182 = -_tmp109 * _tmp131 * _tmp99 - _tmp113 * _tmp117 * _tmp131 +
                         _tmp118 * _tmp136 - _tmp126 * _tmp133 - _tmp146 * _tmp148 + _tmp163;
  const Scalar _tmp183 = std::asinh(_tmp181 * _tmp182);
  const Scalar _tmp184 = Scalar(1.0) * _tmp183;
  const Scalar _tmp185 = Scalar(9.6622558468725703) * _tmp180;
  const Scalar _tmp186 =
      -_tmp183 * _tmp185 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp79), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp82 - 1), Scalar(2))));
  const Scalar _tmp187 = Scalar(0.1034955) * _tmp181;
  const Scalar _tmp188 = _tmp186 * _tmp187;
  const Scalar _tmp189 = std::pow(_tmp180, Scalar(-2));
  const Scalar _tmp190 = _tmp121 * _tmp189;
  const Scalar _tmp191 = (_tmp181 * (-_tmp136 - _tmp148 * _tmp160 + _tmp177) + _tmp182 * _tmp190) /
                         std::sqrt(Scalar(std::pow(_tmp182, Scalar(2)) * _tmp189 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp34 *
      (-_tmp2 * std::cosh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::cosh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp34 -
                    Scalar(8.3196563700000006) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp22 - 1), Scalar(2)) +
                            Scalar(0.057067943376852184) *
                                std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp33 - 1),
                                         Scalar(2)))))));
  _res(1, 0) = _tmp154 * (-Scalar(1.0) * _tmp162 * std::cosh(_tmp153) -
                          (-Scalar(0.1034955) * _tmp155 * _tmp161 +
                           _tmp156 * (-_tmp152 * _tmp158 - _tmp154 * _tmp162)) *
                              std::cosh(_tmp157)) +
               _tmp158 * (-std::sinh(_tmp153) - std::sinh(_tmp157));
  _res(2, 0) = _tmp170 * (-Scalar(1.0) * _tmp179 * std::cosh(_tmp169) -
                          (-Scalar(0.1034955) * _tmp171 * _tmp178 +
                           _tmp172 * (-_tmp168 * _tmp175 - _tmp170 * _tmp179)) *
                              std::cosh(_tmp173)) +
               _tmp175 * (-std::sinh(_tmp169) - std::sinh(_tmp173));
  _res(3, 0) = -_tmp174 * (-std::sinh(_tmp184) - std::sinh(_tmp188)) +
               _tmp185 * (-Scalar(1.0) * _tmp191 * std::cosh(_tmp184) -
                          (Scalar(0.1034955) * _tmp186 * _tmp190 +
                           _tmp187 * (_tmp174 * _tmp183 - _tmp185 * _tmp191)) *
                              std::cosh(_tmp188));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
