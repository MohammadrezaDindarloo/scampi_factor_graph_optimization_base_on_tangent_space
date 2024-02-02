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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl23
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl23(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 657

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (217)
  const Scalar _tmp0 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp1 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp2 = -2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp3 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp4 = 1 - 2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp5 = Scalar(0.20999999999999999) * _tmp2 + Scalar(0.20999999999999999) * _tmp4;
  const Scalar _tmp6 = -_tmp5;
  const Scalar _tmp7 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp8 = 2 * _tmp1 * _tmp7;
  const Scalar _tmp9 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp10 = _tmp3 * _tmp9;
  const Scalar _tmp11 = _tmp10 + _tmp8;
  const Scalar _tmp12 = -Scalar(0.010999999999999999) * _tmp11;
  const Scalar _tmp13 = 2 * _tmp3;
  const Scalar _tmp14 = _tmp13 * _tmp7;
  const Scalar _tmp15 = _tmp1 * _tmp9;
  const Scalar _tmp16 = Scalar(0.20999999999999999) * _tmp14 - Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp17 = _tmp12 + _tmp16;
  const Scalar _tmp18 = _tmp17 + _tmp6;
  const Scalar _tmp19 = _tmp18 + position_vector(0, 0);
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp21 = -_tmp20;
  const Scalar _tmp22 = -2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp23 = Scalar(0.20999999999999999) * _tmp2 + Scalar(0.20999999999999999) * _tmp22 +
                        Scalar(0.20999999999999999);
  const Scalar _tmp24 = _tmp1 * _tmp13;
  const Scalar _tmp25 = _tmp7 * _tmp9;
  const Scalar _tmp26 = _tmp24 - _tmp25;
  const Scalar _tmp27 = -Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp28 = _tmp23 + _tmp27;
  const Scalar _tmp29 = _tmp21 + _tmp28;
  const Scalar _tmp30 = _tmp29 + position_vector(1, 0);
  const Scalar _tmp31 = Scalar(1.0) / (fh1);
  const Scalar _tmp32 = _tmp31 * fv1;
  const Scalar _tmp33 = std::asinh(_tmp32);
  const Scalar _tmp34 = Scalar(9.6622558468725703) * _tmp33;
  const Scalar _tmp35 =
      -_tmp34 * fh1 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp30), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp19 - 1), Scalar(2))));
  const Scalar _tmp36 = Scalar(0.1034955) * _tmp31;
  const Scalar _tmp37 = _tmp35 * _tmp36;
  const Scalar _tmp38 =
      std::pow(Scalar(_tmp0 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp39 = Scalar(1.0) * _tmp33;
  const Scalar _tmp40 = -_tmp23 + _tmp27;
  const Scalar _tmp41 = _tmp20 + _tmp40;
  const Scalar _tmp42 = _tmp41 + position_vector(1, 0);
  const Scalar _tmp43 = _tmp42 + Scalar(8.3888750099999996);
  const Scalar _tmp44 = _tmp12 - _tmp16;
  const Scalar _tmp45 = _tmp44 + _tmp5;
  const Scalar _tmp46 = _tmp45 + position_vector(0, 0);
  const Scalar _tmp47 = _tmp46 + Scalar(-2.5202214700000001);
  const Scalar _tmp48 = std::pow(Scalar(std::pow(_tmp43, Scalar(2)) + std::pow(_tmp47, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp49 = _tmp47 * _tmp48;
  const Scalar _tmp50 = -Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp51 =
      -Scalar(0.010999999999999999) * _tmp22 - Scalar(0.010999999999999999) * _tmp4;
  const Scalar _tmp52 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp53 = _tmp51 + _tmp52;
  const Scalar _tmp54 = _tmp50 + _tmp53;
  const Scalar _tmp55 = _tmp20 + _tmp28;
  const Scalar _tmp56 = _tmp55 + position_vector(1, 0);
  const Scalar _tmp57 = _tmp56 + Scalar(-4.7752063900000001);
  const Scalar _tmp58 = _tmp17 + _tmp5;
  const Scalar _tmp59 = _tmp58 + position_vector(0, 0);
  const Scalar _tmp60 = _tmp59 + Scalar(-2.71799795);
  const Scalar _tmp61 = Scalar(1.0) / (_tmp60);
  const Scalar _tmp62 = _tmp57 * _tmp61;
  const Scalar _tmp63 = _tmp54 * _tmp62;
  const Scalar _tmp64 = _tmp51 - _tmp52;
  const Scalar _tmp65 = _tmp50 + _tmp64;
  const Scalar _tmp66 = _tmp43 * _tmp48;
  const Scalar _tmp67 = -_tmp49 * _tmp63 + _tmp65 * _tmp66;
  const Scalar _tmp68 = Scalar(1.0) / (_tmp49 * _tmp62 - _tmp66);
  const Scalar _tmp69 = Scalar(1.0) * _tmp68;
  const Scalar _tmp70 = _tmp67 * _tmp69;
  const Scalar _tmp71 = Scalar(1.0) * _tmp55;
  const Scalar _tmp72 = -_tmp71;
  const Scalar _tmp73 = Scalar(1.0) / (_tmp41 + _tmp72);
  const Scalar _tmp74 = Scalar(1.0) * _tmp58;
  const Scalar _tmp75 = _tmp73 * (-_tmp45 + _tmp74);
  const Scalar _tmp76 = _tmp49 * _tmp54 - _tmp49 * _tmp65;
  const Scalar _tmp77 = -_tmp69 * _tmp76 + _tmp70 * _tmp75;
  const Scalar _tmp78 = -_tmp50;
  const Scalar _tmp79 = _tmp64 + _tmp78;
  const Scalar _tmp80 = _tmp21 + _tmp40;
  const Scalar _tmp81 = _tmp80 + position_vector(1, 0);
  const Scalar _tmp82 = _tmp81 + Scalar(8.3196563700000006);
  const Scalar _tmp83 = _tmp44 + _tmp6;
  const Scalar _tmp84 = _tmp83 + position_vector(0, 0);
  const Scalar _tmp85 = _tmp84 + Scalar(1.9874742000000001);
  const Scalar _tmp86 = std::pow(Scalar(std::pow(_tmp82, Scalar(2)) + std::pow(_tmp85, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp87 = _tmp82 * _tmp86;
  const Scalar _tmp88 = _tmp85 * _tmp86;
  const Scalar _tmp89 = _tmp62 * _tmp88 - _tmp87;
  const Scalar _tmp90 = _tmp68 * _tmp89;
  const Scalar _tmp91 = _tmp54 * _tmp88;
  const Scalar _tmp92 = -_tmp62 * _tmp91 - _tmp67 * _tmp90 + _tmp79 * _tmp87;
  const Scalar _tmp93 = -_tmp75 * _tmp92 - _tmp76 * _tmp90 - _tmp79 * _tmp88 + _tmp91;
  const Scalar _tmp94 = Scalar(1.0) / (_tmp93);
  const Scalar _tmp95 =
      std::sqrt(Scalar(std::pow(_tmp57, Scalar(2)) + std::pow(_tmp60, Scalar(2))));
  const Scalar _tmp96 = Scalar(1.0) / (_tmp95);
  const Scalar _tmp97 = _tmp61 * _tmp95;
  const Scalar _tmp98 = _tmp97 * (-_tmp55 * _tmp60 * _tmp96 + _tmp57 * _tmp58 * _tmp96);
  const Scalar _tmp99 = _tmp41 * _tmp49 - _tmp45 * _tmp66 + _tmp49 * _tmp98;
  const Scalar _tmp100 = _tmp80 * _tmp88 - _tmp83 * _tmp87 + _tmp88 * _tmp98 - _tmp90 * _tmp99;
  const Scalar _tmp101 = _tmp100 * _tmp94;
  const Scalar _tmp102 = Scalar(1.0) / (_tmp100);
  const Scalar _tmp103 = _tmp102 * _tmp93;
  const Scalar _tmp104 = _tmp103 * (-_tmp101 * _tmp77 - _tmp69 * _tmp99);
  const Scalar _tmp105 = _tmp104 + _tmp77;
  const Scalar _tmp106 = _tmp92 * _tmp94;
  const Scalar _tmp107 = _tmp72 + _tmp80;
  const Scalar _tmp108 = _tmp107 * _tmp75;
  const Scalar _tmp109 = Scalar(1.0) / (-_tmp108 + _tmp74 - _tmp83);
  const Scalar _tmp110 = _tmp107 * _tmp109;
  const Scalar _tmp111 = _tmp104 * _tmp110 - _tmp105 * _tmp106 - _tmp70;
  const Scalar _tmp112 = Scalar(1.0) * _tmp73;
  const Scalar _tmp113 = Scalar(1.0) * _tmp109;
  const Scalar _tmp114 = _tmp30 + Scalar(-4.8333311099999996);
  const Scalar _tmp115 = _tmp19 + Scalar(1.79662371);
  const Scalar _tmp116 =
      std::pow(Scalar(std::pow(_tmp114, Scalar(2)) + std::pow(_tmp115, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp117 = _tmp114 * _tmp116;
  const Scalar _tmp118 = Scalar(1.0) * _tmp117 * (_tmp104 * _tmp113 - _tmp111 * _tmp112);
  const Scalar _tmp119 = Scalar(1.0) * _tmp102;
  const Scalar _tmp120 = _tmp103 * _tmp113;
  const Scalar _tmp121 = _tmp107 * _tmp120 - _tmp119 * _tmp92;
  const Scalar _tmp122 = _tmp115 * _tmp116;
  const Scalar _tmp123 = _tmp117 * _tmp18 - _tmp122 * _tmp29;
  const Scalar _tmp124 = Scalar(1.0) * _tmp123;
  const Scalar _tmp125 = _tmp124 * (-_tmp112 * _tmp121 + _tmp120);
  const Scalar _tmp126 = _tmp53 + _tmp78;
  const Scalar _tmp127 = _tmp126 * fh1;
  const Scalar _tmp128 = -_tmp117 * _tmp127 - Scalar(5.1796800000000003) * _tmp26 - _tmp29 * fv1;
  const Scalar _tmp129 = _tmp73 * (_tmp108 * _tmp113 + Scalar(1.0));
  const Scalar _tmp130 = _tmp113 * _tmp75;
  const Scalar _tmp131 = -Scalar(1.0) * _tmp129 + Scalar(1.0) * _tmp130;
  const Scalar _tmp132 = _tmp62 * _tmp68;
  const Scalar _tmp133 = _tmp132 * _tmp67 + _tmp63;
  const Scalar _tmp134 = _tmp132 * _tmp76 - _tmp133 * _tmp75 - _tmp54;
  const Scalar _tmp135 = _tmp103 * (-_tmp101 * _tmp134 + _tmp132 * _tmp99 - _tmp98);
  const Scalar _tmp136 = _tmp134 + _tmp135;
  const Scalar _tmp137 = -_tmp106 * _tmp136 + _tmp110 * _tmp135 + _tmp133;
  const Scalar _tmp138 = Scalar(1.0) * _tmp122 * (-_tmp112 * _tmp137 + _tmp113 * _tmp135);
  const Scalar _tmp139 = Scalar(5.1796800000000003) * _tmp11 + _tmp122 * _tmp127 + _tmp18 * fv1;
  const Scalar _tmp140 = _tmp107 * _tmp73;
  const Scalar _tmp141 = Scalar(1.0) * _tmp113 * _tmp140 - Scalar(1.0) * _tmp113;
  const Scalar _tmp142 = _tmp71 * _tmp75 + _tmp74;
  const Scalar _tmp143 = _tmp109 * _tmp142;
  const Scalar _tmp144 = 0;
  const Scalar _tmp145 = _tmp73 * (-_tmp106 * _tmp144 - _tmp107 * _tmp143 + _tmp72);
  const Scalar _tmp146 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp147 =
      _tmp118 * fh1 + _tmp125 * fh1 + _tmp128 * _tmp131 + _tmp138 * fh1 + _tmp139 * _tmp141 +
      Scalar(1.0) * _tmp146 * (-_tmp113 * _tmp142 - Scalar(1.0) * _tmp145 + Scalar(1.0));
  const Scalar _tmp148 = _tmp102 * _tmp69 * _tmp89;
  const Scalar _tmp149 = _tmp123 * _tmp97 * (_tmp119 * _tmp88 - _tmp148 * _tmp49);
  const Scalar _tmp150 = _tmp144 * _tmp94;
  const Scalar _tmp151 = _tmp89 * _tmp94;
  const Scalar _tmp152 = -_tmp105 * _tmp151 + Scalar(1.0);
  const Scalar _tmp153 = _tmp49 * _tmp68;
  const Scalar _tmp154 = _tmp105 * _tmp94;
  const Scalar _tmp155 = _tmp117 * _tmp97 * (_tmp152 * _tmp153 + _tmp154 * _tmp88);
  const Scalar _tmp156 = _tmp136 * _tmp94;
  const Scalar _tmp157 = -_tmp136 * _tmp151 - _tmp62;
  const Scalar _tmp158 = _tmp122 * _tmp97 * (_tmp153 * _tmp157 + _tmp156 * _tmp88 + Scalar(1.0));
  const Scalar _tmp159 = -_tmp146 * _tmp97 * (-_tmp150 * _tmp49 * _tmp90 + _tmp150 * _tmp88) -
                         _tmp149 * fh1 - _tmp155 * fh1 - _tmp158 * fh1;
  const Scalar _tmp160 = Scalar(1.0) / (_tmp159);
  const Scalar _tmp161 = std::asinh(_tmp147 * _tmp160);
  const Scalar _tmp162 = Scalar(9.6622558468725703) * _tmp159;
  const Scalar _tmp163 =
      -_tmp161 * _tmp162 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp56), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp59), Scalar(2))));
  const Scalar _tmp164 = Scalar(0.1034955) * _tmp160;
  const Scalar _tmp165 = _tmp163 * _tmp164;
  const Scalar _tmp166 = Scalar(1.0) * _tmp161;
  const Scalar _tmp167 = -_tmp149 - _tmp155 - _tmp158;
  const Scalar _tmp168 = Scalar(9.6622558468725703) * _tmp167;
  const Scalar _tmp169 = std::pow(_tmp159, Scalar(-2));
  const Scalar _tmp170 = _tmp117 * _tmp126;
  const Scalar _tmp171 = _tmp122 * _tmp126;
  const Scalar _tmp172 = _tmp167 * _tmp169;
  const Scalar _tmp173 = (-_tmp147 * _tmp172 + _tmp160 * (_tmp118 + _tmp125 - _tmp131 * _tmp170 +
                                                          _tmp138 + _tmp141 * _tmp171)) /
                         std::sqrt(Scalar(std::pow(_tmp147, Scalar(2)) * _tmp169 + 1));
  const Scalar _tmp174 = _tmp117 * _tmp152 * _tmp68;
  const Scalar _tmp175 = _tmp122 * _tmp157 * _tmp68;
  const Scalar _tmp176 = _tmp123 * _tmp148;
  const Scalar _tmp177 = _tmp146 * _tmp150;
  const Scalar _tmp178 = _tmp174 * fh1 + _tmp175 * fh1 - _tmp176 * fh1 - _tmp177 * _tmp90;
  const Scalar _tmp179 = std::pow(_tmp178, Scalar(-2));
  const Scalar _tmp180 = _tmp174 + _tmp175 - _tmp176;
  const Scalar _tmp181 = _tmp179 * _tmp180;
  const Scalar _tmp182 = _tmp122 * _tmp73;
  const Scalar _tmp183 = _tmp137 * _tmp182;
  const Scalar _tmp184 = _tmp113 * _tmp139;
  const Scalar _tmp185 = _tmp111 * _tmp117 * _tmp73;
  const Scalar _tmp186 = _tmp121 * _tmp123 * _tmp73;
  const Scalar _tmp187 = _tmp128 * _tmp129 - _tmp140 * _tmp184 + _tmp145 * _tmp146 + _tmp183 * fh1 +
                         _tmp185 * fh1 + _tmp186 * fh1;
  const Scalar _tmp188 = Scalar(1.0) / (_tmp178);
  const Scalar _tmp189 = std::asinh(_tmp187 * _tmp188);
  const Scalar _tmp190 = Scalar(9.6622558468725703) * _tmp180;
  const Scalar _tmp191 = Scalar(9.6622558468725703) * _tmp178;
  const Scalar _tmp192 =
      (-_tmp181 * _tmp187 + _tmp188 * (-_tmp107 * _tmp113 * _tmp126 * _tmp182 - _tmp129 * _tmp170 +
                                       _tmp183 + _tmp185 + _tmp186)) /
      std::sqrt(Scalar(_tmp179 * std::pow(_tmp187, Scalar(2)) + 1));
  const Scalar _tmp193 = Scalar(0.1034955) * _tmp188;
  const Scalar _tmp194 =
      -_tmp189 * _tmp191 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp46), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp42 - 1), Scalar(2))));
  const Scalar _tmp195 = _tmp193 * _tmp194;
  const Scalar _tmp196 = Scalar(1.0) * _tmp189;
  const Scalar _tmp197 = _tmp117 * _tmp154;
  const Scalar _tmp198 = _tmp102 * _tmp124;
  const Scalar _tmp199 = _tmp122 * _tmp156;
  const Scalar _tmp200 = _tmp177 + _tmp197 * fh1 + _tmp198 * fh1 + _tmp199 * fh1;
  const Scalar _tmp201 = Scalar(1.0) / (_tmp200);
  const Scalar _tmp202 = _tmp104 * _tmp109 * _tmp117;
  const Scalar _tmp203 = _tmp120 * _tmp123;
  const Scalar _tmp204 = _tmp109 * _tmp122 * _tmp135;
  const Scalar _tmp205 = -_tmp128 * _tmp130 + _tmp143 * _tmp146 + _tmp184 - _tmp202 * fh1 -
                         _tmp203 * fh1 - _tmp204 * fh1;
  const Scalar _tmp206 = std::asinh(_tmp201 * _tmp205);
  const Scalar _tmp207 = Scalar(9.6622558468725703) * _tmp200;
  const Scalar _tmp208 =
      -_tmp206 * _tmp207 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp81 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp84 - 1), Scalar(2))));
  const Scalar _tmp209 = Scalar(0.1034955) * _tmp201;
  const Scalar _tmp210 = _tmp208 * _tmp209;
  const Scalar _tmp211 = Scalar(1.0) * _tmp206;
  const Scalar _tmp212 = _tmp197 + _tmp198 + _tmp199;
  const Scalar _tmp213 = Scalar(9.6622558468725703) * _tmp212;
  const Scalar _tmp214 = std::pow(_tmp200, Scalar(-2));
  const Scalar _tmp215 = _tmp212 * _tmp214;
  const Scalar _tmp216 =
      (_tmp201 * (_tmp113 * _tmp171 + _tmp130 * _tmp170 - _tmp202 - _tmp203 - _tmp204) -
       _tmp205 * _tmp215) /
      std::sqrt(Scalar(std::pow(_tmp205, Scalar(2)) * _tmp214 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = -Scalar(8.3700199099999999) * _tmp31 -
               Scalar(9.6622558468725703) * fh1 *
                   (-Scalar(1.0) * _tmp0 * _tmp38 * fv1 * std::sinh(_tmp39) -
                    Scalar(0.86625939559540499) * _tmp0 -
                    (-Scalar(0.1034955) * _tmp0 * _tmp35 +
                     _tmp36 * (Scalar(9.6622558468725703) * _tmp32 * _tmp38 - _tmp34)) *
                        std::sinh(_tmp37)) +
               Scalar(9.6622558468725703) * std::cosh(_tmp37) -
               Scalar(9.6622558468725703) * std::cosh(_tmp39);
  _res(1, 0) =
      -_tmp162 *
          (-Scalar(0.86565325453551001) * _tmp172 + Scalar(1.0) * _tmp173 * std::sinh(_tmp166) -
           (-Scalar(0.1034955) * _tmp163 * _tmp172 +
            _tmp164 * (-_tmp161 * _tmp168 - _tmp162 * _tmp173)) *
               std::sinh(_tmp165)) -
      _tmp168 * (Scalar(0.86565325453551001) * _tmp160 - std::cosh(_tmp165) + std::cosh(_tmp166));
  _res(2, 0) =
      -_tmp190 * (Scalar(0.87653584775870996) * _tmp188 - std::cosh(_tmp195) + std::cosh(_tmp196)) -
      _tmp191 *
          (-Scalar(0.87653584775870996) * _tmp181 + Scalar(1.0) * _tmp192 * std::sinh(_tmp196) -
           (-Scalar(0.1034955) * _tmp181 * _tmp194 +
            _tmp193 * (-_tmp189 * _tmp190 - _tmp191 * _tmp192)) *
               std::sinh(_tmp195));
  _res(3, 0) =
      -_tmp207 *
          (-Scalar(0.87679799772039002) * _tmp215 + Scalar(1.0) * _tmp216 * std::sinh(_tmp211) -
           (-Scalar(0.1034955) * _tmp208 * _tmp215 +
            _tmp209 * (-_tmp206 * _tmp213 - _tmp207 * _tmp216)) *
               std::sinh(_tmp210)) -
      _tmp213 * (Scalar(0.87679799772039002) * _tmp201 - std::cosh(_tmp210) + std::cosh(_tmp211));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
