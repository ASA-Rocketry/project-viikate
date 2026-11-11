# MATLAB R2026b workstation for the Viikate plant model (simulation/matlab).
# Extends MathWorks' prebuilt image with the products closedmodel.slx and the
# SIL codegen plan need (docs/local/sim_sil_codegen_plan.md). The image carries
# no license: sign in on first start, or set MLM_LICENSE_FILE for a network
# license.
#
# From the repository root:
#   docker build -f simulation/matlab/Dockerfile -t viikate-matlab simulation/matlab

ARG MATLAB_TAG=r2026b
FROM mathworks/matlab:${MATLAB_TAG}

ARG MATLAB_RELEASE=R2026b
ARG MATLAB_PRODUCTS="Simulink Aerospace_Blockset MATLAB_Coder Simulink_Coder Embedded_Coder"

# mpm adds the products to the MATLAB already installed in the base image and is
# deleted afterwards so it does not stay in the layer. On failure, print its log.
RUN wget -q https://www.mathworks.com/mpm/glnxa64/mpm \
    && chmod +x mpm \
    && sudo HOME=${HOME} ./mpm install \
        --release=${MATLAB_RELEASE} \
        --destination=/opt/matlab/${MATLAB_RELEASE} \
        --products ${MATLAB_PRODUCTS} \
    || (echo "mpm install failed, log follows:" && cat /tmp/mathworks_root.log && false) \
    && sudo rm -rf mpm /tmp/mathworks_root.log
