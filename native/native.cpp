#include <Python.h>
#include <windows.h>
#ifdef min
#undef min
#undef max
#endif

#include <set>
#include <vector>
#include <numpy/arrayobject.h>
#include <maya/MGlobal.h>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MQuaternion.h>
#include <maya/MTransformationMatrix.h>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Eigen>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <osqp/osqp.h>
#include <osqp/auxil.h>

#pragma region Macros
inline void SET_LONG2(PyArrayObject* m, long i0, long i1, long v)
{
    *reinterpret_cast<npy_longlong*>(PyArray_GETPTR2(m, i0, i1)) = static_cast<npy_longlong>(v);
}
inline void SET_DOUBLE2(PyArrayObject* m, long i0, long i1, double v)
{
    *reinterpret_cast<npy_float64*>(PyArray_GETPTR2(m, i0, i1)) = static_cast<npy_float64>(v);
}
inline void SET_DOUBLE4(PyArrayObject* m, long i0, long i1, long i2, long i3, double v)
{
    *reinterpret_cast<npy_float64*>(PyArray_GETPTR4(m, i0, i1, i2, i3)) = static_cast<npy_float64>(v);
}
inline long GET_LONG2(PyArrayObject* m, long i0, long i1)
{
    return static_cast<long>(*reinterpret_cast<npy_longlong*>(PyArray_GETPTR2(m, i0, i1)));
}
inline double GET_DOUBLE2(PyArrayObject* m, long i0, long i1)
{
    return static_cast<double>(*reinterpret_cast<npy_float64*>(PyArray_GETPTR2(m, i0, i1)));
}
inline double GET_DOUBLE3(PyArrayObject* m, long i0, long i1, long i2)
{
    return static_cast<double>(*reinterpret_cast<npy_float64*>(PyArray_GETPTR3(m, i0, i1, i2)));
}
inline double GET_DOUBLE4(PyArrayObject* m, long i0, long i1, long i2, long i3)
{
    return static_cast<double>(*reinterpret_cast<npy_float64*>(PyArray_GETPTR4(m, i0, i1, i2, i3)));
}
#pragma endregion

struct Input
{
    //! number of model vertices
    long numVertices;
    //! number of sample frames
    long numSamples;
    //! desirable number of joints
    long numJoints;
    //! local neighbors (0 / 1 / 2)
    long numRings;
    //! rest/bind shape (numVertices)
    std::vector<MPoint> restShape;
    //! samples (numSamples x numVertices)
    std::vector<MPoint> sample;
    //! neighbor information
    std::vector<std::vector<std::pair<int, double>>> neighbor;
    //! model normalization matrix
    MMatrix normalizer;

    Input() :
        numVertices(0), numSamples(0), numJoints(0), normalizer(MMatrix::identity)
	{
	}
	~Input()
	{
		restShape.clear();
		sample.clear();
        neighbor.clear();
	}
};

struct Output
{
    //! number of inserted joints
    long numJoints;
    //! number of skinning joint index (max influence)
    long numIndices;
    //! skinning weight (numVertices x numIndices)
    std::vector<double> skinWeight;
    //! skinning joint index (numVertices x numIndices)
    std::vector<long> skinIndex;
    //! skinning matrix (numJoints x numSamples)
    std::vector<MTransformationMatrix> skinMatrix;
    //! rest joint position (numJoints)
    std::vector<MPoint> restJointPos;
    //! clusters each vertex belongs
    std::vector<std::vector<int>> vertCluster;
	
	Output() :
        numJoints(0), numIndices(0)
	{
	}
	~Output()
	{
		skinWeight.clear();
		skinIndex.clear();
		skinMatrix.clear();
        restJointPos.clear();
	}
};

#pragma region Subroutines for joint transformation
MTransformationMatrix calcRegistrationT(long numPoints, std::vector<MPoint>::const_iterator ps, std::vector<MPoint>::const_iterator pd)
{
    MTransformationMatrix transform;
    MPoint cs = MVector::zero, cd = MVector::zero;
    std::vector<MPoint>::const_iterator sit = ps;
    std::vector<MPoint>::const_iterator dit = pd;
    for (long i = 0; i < numPoints; ++i, ++sit, ++dit)
    {
        cs += *sit;
        cd += *dit;
    }
    transform.setTranslation((cd - cs) / static_cast<double>(numPoints), MSpace::kTransform);
    return transform;
}

MTransformationMatrix calcRegistrationRT(long numPoints, std::vector<MPoint>::const_iterator ps, std::vector<MPoint>::const_iterator pd)
{
    MTransformationMatrix transform;
    MPoint cs = MVector::zero, cd = MVector::zero;
    std::vector<MPoint>::const_iterator sit = ps;
    std::vector<MPoint>::const_iterator dit = pd;
    for (long i = 0; i < numPoints; ++i, ++sit, ++dit)
    {
        cs += *sit;
        cd += *dit;
    }
    cs = cs / static_cast<double>(numPoints);
    cd = cd / static_cast<double>(numPoints);
    if (numPoints < 3)
    {
        transform.setTranslation(cd - cs, MSpace::kTransform);
        return transform;
    }

    Eigen::Matrix<double, 4, 4> moment;
    double sxx = 0, sxy = 0, sxz = 0, syx = 0, syy = 0, syz = 0, szx = 0, szy = 0, szz = 0;
    sit = ps;
    dit = pd;
    for (long i = 0; i < numPoints; ++i, ++sit, ++dit)
    {
        sxx += (sit->x - cs.x) * (dit->x - cd.x);
        sxy += (sit->x - cs.x) * (dit->y - cd.y);
        sxz += (sit->x - cs.x) * (dit->z - cd.z);
        syx += (sit->y - cs.y) * (dit->x - cd.x);
        syy += (sit->y - cs.y) * (dit->y - cd.y);
        syz += (sit->y - cs.y) * (dit->z - cd.z);
        szx += (sit->z - cs.z) * (dit->x - cd.x);
        szy += (sit->z - cs.z) * (dit->y - cd.y);
        szz += (sit->z - cs.z) * (dit->z - cd.z);
    }
    moment(0, 0) = sxx + syy + szz;
    moment(0, 1) = syz - szy;        moment(1, 0) = moment(0, 1);
    moment(0, 2) = szx - sxz;        moment(2, 0) = moment(0, 2);
    moment(0, 3) = sxy - syx;        moment(3, 0) = moment(0, 3);
    moment(1, 1) = sxx - syy - szz;
    moment(1, 2) = sxy + syx;        moment(2, 1) = moment(1, 2);
    moment(1, 3) = szx + sxz;        moment(3, 1) = moment(1, 3);
    moment(2, 2) = -sxx + syy - szz;
    moment(2, 3) = syz + szy;        moment(3, 2) = moment(2, 3);
    moment(3, 3) = -sxx - syy + szz;

    if (moment.norm() > 0)
    {
        Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> es(moment);
        long maxi = 0;
        for (long i = 1; i < 4; ++i)
        {
            if (es.eigenvalues()(maxi).real() < es.eigenvalues()(i).real())
            {
                maxi = i;
            }
        }
        transform.setRotationQuaternion(
            es.eigenvectors()(1, maxi).real(),
            es.eigenvectors()(2, maxi).real(),
            es.eigenvectors()(3, maxi).real(),
            es.eigenvectors()(0, maxi).real());
    }
    MPoint cs0 = cs * transform.asMatrix();
    transform.setTranslation(cd - cs0, MSpace::kTransform);
    return transform;
}

MTransformationMatrix calcRegistrationSRT(long numPoints, std::vector<MPoint>::const_iterator ps, std::vector<MPoint>::const_iterator pd)
{
    const long numIterations = 50;
    const double threshold = 1.0e-8;
    const double scaleLowerBound = 1.0e-3;
    MTransformationMatrix transform = calcRegistrationRT(numPoints, ps, pd);

    MVector scale(1.0, 1.0, 1.0);
    MVector denom(0.0, 0.0, 0.0);
    std::vector<MPoint>::const_iterator sit = ps;
    for (long i = 0; i < numPoints; ++i, ++sit)
    {
        denom[0] += (*sit)[0] * (*sit)[0];
        denom[1] += (*sit)[1] * (*sit)[1];
        denom[2] += (*sit)[2] * (*sit)[2];
    }
    std::vector<MPoint> sstm(numPoints);
    sit = ps;
    for (long i = 0; i < numPoints; ++i, ++sit)
    {
        sstm[i][0] = (*sit)[0];
        sstm[i][1] = (*sit)[1];
        sstm[i][2] = (*sit)[2];
    }
    std::vector<MPoint> dstm(numPoints);
    for (long l = 0; l < numIterations; ++l)
    {
        MMatrix im = transform.asMatrix().inverse();
        std::vector<MPoint>::const_iterator dit = pd;
        for (long i = 0; i < numPoints; ++i, ++dit)
        {
            dstm[i] = *dit * im;
        }
        for (long d = 0; d < 3; ++d)
        {
            double ds = 0.0;
            sit = ps;
            for (long i = 0; i < numPoints; ++i, ++sit)
            {
                ds += (*sit)[d] * dstm[i][d];
            }
            scale[d] = denom[d] < threshold ? 1.0 : ds / denom[d];
            sit = ps;
            for (long i = 0; i < numPoints; ++i, ++sit)
            {
                sstm[i][d] = scale[d] * (*sit)[d];
            }
        }
        transform = calcRegistrationRT(numPoints, sstm.begin(), pd);
    }
    double sv[3] = { scale[0], scale[1], scale[2] };
    transform.setScale(sv, MSpace::kTransform);
    return transform;
}

typedef MTransformationMatrix(*RegistrationFunc)(long numPoints, std::vector<MPoint>::const_iterator ps, std::vector<MPoint>::const_iterator pd);

RegistrationFunc registrationFuncs[3] = {
    calcRegistrationT,
    calcRegistrationRT,
    calcRegistrationSRT };

void computeSamplePoints(std::vector<MPoint>& sample, long sid, long joint, const Output& output, const Input& input)
{
    const long numVertices = input.numVertices;
    const long& numIndices = output.numIndices;

    for (long v = 0; v < numVertices; ++v)
    {
        sample[v] = input.sample[sid * numVertices + v];
        const MPoint& s = input.restShape[v];
        for (long i = 0; i < numIndices; ++i)
        {
            const long jnt = output.skinIndex[v * numIndices + i];
            if (jnt >= 0 && jnt != joint)
            {
                const double w = output.skinWeight[v * numIndices + i];
                const MTransformationMatrix& at = output.skinMatrix[sid * input.numJoints + jnt];
                sample[v] -= w * (s * at.asMatrix());
            }
        }
    }
}

void subtractCentroid(std::vector<MPoint>& model, std::vector<MPoint>& sample, MPoint& corModel, MPoint& corSample, const Eigen::VectorXd& weight, const Output& output, const Input& input)
{
    const long numVertices = input.numVertices;

    double wsqsum = 0;
    corModel = MVector::zero;
    corSample = MVector::zero;
    for (long v = 0; v < numVertices; ++v)
    {
        const double w = weight[v];
        corModel += w * w * input.restShape[v];
        corSample += w * sample[v];
        wsqsum += w * w;
    }
    corModel = corModel / wsqsum;
    corSample = corSample / wsqsum;
    for (long v = 0; v < numVertices; ++v)
    {
        model[v] = weight[v] * (input.restShape[v] - corModel);
        sample[v] -= weight[v] * corSample;
    }
}
#pragma endregion

#pragma region Bone transformation
class JointTransformUpdator
{
private:
    Output* output;
    const Input* input;
    const Eigen::VectorXd* weight;
    int transformType;
    long joint;

public:
    JointTransformUpdator(Output* output_, const Input* input_,
        const Eigen::VectorXd* weight_, int joint_, int transformType_)
        : output(output_), input(input_),
        weight(weight_), joint(joint_), transformType(transformType_)
    {
    }
    void operator ()(const tbb::blocked_range<int>& range) const
    {
        std::vector<MPoint> model(input->numVertices);
        std::vector<MPoint> sample(input->numVertices);
        for (long s = range.begin(); s != range.end(); ++s)
        {
            if (s == 0)
            {
                output->skinMatrix[joint] = MTransformationMatrix::identity;
                continue;
            }
            computeSamplePoints(sample, s, joint, *output, *input);
            MPoint corModel(0, 0, 0), corSample(0, 0, 0);
            subtractCentroid(model, sample, corModel, corSample, *weight, *output, *input);
            MTransformationMatrix transform = registrationFuncs[transformType](model.size(), model.begin(), sample.begin());
            MVector d = corSample - corModel * transform.asMatrix();
            transform.setTranslation(d + transform.getTranslation(MSpace::kTransform), MSpace::kTransform);
            output->skinMatrix[s * input->numJoints + joint] = transform;
        }
    }
};

void updateJointTransformProc(Output& output, int transformType, const Input& input, long selection = -1)
{
    const long numVertices = input.numVertices;
    const long numSamples  = input.numSamples;
    const long numJoints    = output.numJoints;
    const long numIndices  = output.numIndices;

    Eigen::VectorXd weight = Eigen::VectorXd::Zero(numVertices);
    long begin = selection < 0 ? 0 : selection;
    long end = selection < 0 ? numJoints : selection + 1;
    for (long joint = begin; joint < end; ++joint)
    {
        for (long v = 0; v < numVertices; ++v)
        {
            weight[v] = 0.0;
            for (long i = 0; i < numIndices; ++i)
            {
                long jnt = output.skinIndex[v * numIndices + i];
                if (jnt == joint)
                {
                    weight[v] = output.skinWeight[v * numIndices + i];
                    break;
                }
            }
        }
        double wsqsum = weight.dot(weight);
        if (wsqsum > 1.0e-8)
        {
            tbb::blocked_range<int> blockedRange(0, numSamples);
            JointTransformUpdator transformUpdator(&output, &input, &weight, joint, transformType);
            tbb::parallel_for(blockedRange, transformUpdator);
        }
        else
        {
            for (int s = 0; s < numSamples; ++s)
            {
                output.skinMatrix[s * input.numJoints + joint] = MTransformationMatrix::identity;
            }
            char buf[256];
            sprintf(buf, "Fixed joint #%d", joint);
            MGlobal::displayInfo(buf);
        }
    }
}

static PyObject* updateJointTransform(PyObject *self, PyObject *args)
{
    PyObject* pin            = PyTuple_GET_ITEM(args, 0);
    PyObject* pout           = PyTuple_GET_ITEM(args, 1);
    const long transformType = PyInt_AsLong(PyTuple_GET_ITEM(args, 2));
    Input* const input   = reinterpret_cast<Input*>(PyCapsule_GetPointer(pin, "SSDSInput"));
    Output* const output = reinterpret_cast<Output*>(PyCapsule_GetPointer(pout, "SSDSOutput"));
    updateJointTransformProc(*output, transformType, *input);
    return Py_None;
}
#pragma endregion 

#pragma region Skinning weight
Eigen::VectorXd solveWeightQP(Eigen::SparseMatrix<c_float, 0, c_int>& pm, Eigen::VectorXd& qv)
{
    const int n = static_cast<int>(pm.rows());
    Eigen::SparseMatrix<c_float, 0, c_int> a(n + 1, n);
    std::vector<c_float> q(n), l(n + 1, 0.0), u(n + 1, 1.0);
    for (int i = 0; i < n; ++i)
    {
        a.coeffRef(0, i) = 1.0;
        a.coeffRef(i + 1, i) = 1.0;
        q[i] = qv[i];
    }
    l[0] = u[0] = 1.0;
    a.makeCompressed();
    pm.makeCompressed();

    // Problem settings
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.polish = 0;
    settings.adaptive_rho = 0;
    settings.warm_start = 0;
    settings.eps_abs = 1.0e-12; // experimentally set
    settings.eps_rel = 1.0e-12; // experimentally set

    OSQPData osqpData;
    osqpData.n = n;
    osqpData.m = n + 1;
    osqpData.P = csc_matrix(n, n, pm.nonZeros(), pm.valuePtr(), pm.innerIndexPtr(), pm.outerIndexPtr());
    osqpData.q = q.data();
    osqpData.A = csc_matrix(n + 1, n, a.nonZeros(), a.valuePtr(), a.innerIndexPtr(), a.outerIndexPtr());
    osqpData.l = l.data();
    osqpData.u = u.data();

    OSQPWorkspace* osqpWork = osqp_setup(&osqpData, &settings);
    c_int retval = osqp_solve(osqpWork);
    settings.rho = compute_rho_estimate(osqpWork);
    Eigen::VectorXd weight(n);
    for (int i = 0; i < n; ++i)
    {
        weight[i] = osqpWork->solution->x[i];
    }
    osqp_cleanup(osqpWork);

    settings.warm_start = 1;
    osqpWork = osqp_setup(&osqpData, &settings);
    osqp_warm_start_x(osqpWork, weight.data());
    retval = osqp_solve(osqpWork);
    for (int i = 0; i < n; ++i)
    {
        weight[i] = osqpWork->solution->x[i];
    }
    osqp_cleanup(osqpWork);
    c_free(osqpData.A);
    c_free(osqpData.P);
    return weight;
}

class SkinWeightUpdator
{
private:
    Output* output;
    const Input* input;
    const int numIndices;
    const double smoothness;
    std::vector<double> prevWeight;
    std::vector<long> prevIndex;
public:
    SkinWeightUpdator(Output* output_, const Input* input_,
        int numMaxInfluences_, double smoothness_)
        : output(output_), input(input_),
        numIndices(numMaxInfluences_), smoothness(smoothness_ == 0 ? 0 : std::pow(10.0, smoothness_ - 8))
    {
        prevWeight = output->skinWeight;
        prevIndex = output->skinIndex;
    }
    void operator ()(const tbb::blocked_range<int>& range) const
    {
        const int numVertices = input->numVertices;
        const int numSamples  = input->numSamples;
        const int numJoints   = output->numJoints;
        const double epsilon  = 1.0e-3;

        Eigen::MatrixXd sbasis   = Eigen::MatrixXd::Zero(numIndices, numSamples * 3);
        Eigen::VectorXd sample   = Eigen::VectorXd::Zero(numSamples * 3);
        Eigen::VectorXd slaplacm = Eigen::VectorXd::Zero(numIndices);
        Eigen::VectorXd slaplacv = Eigen::VectorXd::Zero(numIndices);
        for (int v = range.begin(); v != range.end(); ++v)
        {
            const std::vector<int>& vertCluster = output->vertCluster[v];
            const int numActJoints = vertCluster.size();
            Eigen::MatrixXd basis   = Eigen::MatrixXd::Zero(numActJoints, numSamples * 3);
            Eigen::VectorXd laplacm = Eigen::VectorXd::Zero(numActJoints);
            Eigen::VectorXd laplacv = Eigen::VectorXd::Zero(numActJoints);

            for (int i = 0; i < numIndices; ++i)
            {
                output->skinIndex[v * numIndices + i] = -1;
                output->skinWeight[v * numIndices + i] = 0;
            }
            // Laplacian term
            double lsum = 0.0;
            laplacv.setZero();

            for (auto it = input->neighbor[v].begin(); it != input->neighbor[v].end(); ++it)
            {
                const int nvid = it->first;
                lsum -= it->second;
                for (int i = 0; i < numIndices; ++i)
                {
                    const int nbid = prevIndex[nvid * numIndices + i];
                    auto nbit = std::find(output->vertCluster[v].begin(), output->vertCluster[v].end(), nbid);
                    if (nbid >= 0 && nbit != output->vertCluster[v].end())
                    {
                        laplacv[nbit - output->vertCluster[v].begin()] -= it->second * prevWeight[nvid * numIndices + i];
                    }
                }
            }
            laplacm.setConstant(smoothness * lsum);
            laplacv *= -smoothness;
            

            // variables
            const MPoint& restVertex = input->restShape[v];
            for (int s = 0; s < numSamples; ++s)
            {
                for (int j = 0; j < numActJoints; ++j)
                {
                    const int jid = vertCluster[j];
                    const MTransformationMatrix& rt = output->skinMatrix[s * numJoints + jid];
                    MPoint tv = restVertex * rt.asMatrix();
                    basis(j, s * 3 + 0) = tv.x;
                    basis(j, s * 3 + 1) = tv.y;
                    basis(j, s * 3 + 2) = tv.z;
                }
                sample[s * 3 + 0] = input->sample[s * numVertices + v].x;
                sample[s * 3 + 1] = input->sample[s * numVertices + v].y;
                sample[s * 3 + 2] = input->sample[s * numVertices + v].z;
            }
            // optimization
            Eigen::SparseMatrix<c_float, 0, c_int> gm =
                (basis * basis.transpose()
                    + laplacm.asDiagonal().toDenseMatrix()
                    ).sparseView();
            gm.makeCompressed();
            Eigen::VectorXd gv = -basis * sample + laplacv;
            double scale = std::max(std::abs(gv.minCoeff()), std::abs(gv.maxCoeff()));

            Eigen::VectorXd weight = solveWeightQP(gm, gv);
            double weightSum = 0;
            for (int i = 0; i < numIndices; ++i)
            {
                double maxw = epsilon;
                int bestjoint = -1;
                for (int j = 0; j < numActJoints; ++j)
                {
                    if (weight[j] > maxw)
                    {
                        maxw = weight[j];
                        bestjoint = j;
                    }
                }
                if (bestjoint < 0)
                {
                    break;
                }
                output->skinIndex[v * numIndices + i] = bestjoint;
                output->skinWeight[v * numIndices + i] = maxw;
                weightSum += maxw;
                weight[bestjoint] = -1.0;
            }
            // subproblem
            if (weightSum < 1.0)
            {
                sbasis.setZero();
                slaplacm.setZero();
                slaplacv.setZero();
                for (int i = 0; i < numIndices; ++i)
                {
                    int j = output->skinIndex[v * numIndices + i];
                    if (j >= 0)
                    {
                        for (int s = 0; s < numSamples * 3; ++s)
                        {
                            sbasis(i, s) = basis(j, s);
                        }
                        slaplacm[i] = laplacm[j];
                        slaplacv[i] = laplacv[j];
                    }
                }
                Eigen::SparseMatrix<c_float, 0, c_int> sgm =
                    (sbasis * sbasis.transpose()
                        + slaplacm.asDiagonal().toDenseMatrix()
                        ).sparseView();
                Eigen::VectorXd sgv = -sbasis * sample + slaplacv;
                sgm.makeCompressed();
                weight = solveWeightQP(sgm, sgv);
                weightSum = 0;
                for (int i = 0; i < numIndices; ++i)
                {
                    if (weight[i] < epsilon)
                    {
                        output->skinWeight[v * numIndices + i] = 0;
                        output->skinIndex[v * numIndices + i] = -1;
                    }
                    else
                    {
                        output->skinWeight[v * numIndices + i] = weight[i];
                        weightSum += weight[i];
                    }
                }
            }
            for (int i = 0; i < numIndices; ++i)
            {
                output->skinIndex[v * numIndices + i] = output->skinIndex[v * numIndices + i] < 0
                    ? -1 : vertCluster[output->skinIndex[v * numIndices + i]];
                output->skinWeight[v * numIndices + i] /= weightSum;
            }
        }
    }
};

static PyObject* updateSkinWeight(PyObject *self, PyObject *args)
{
    PyObject* pin        = PyTuple_GET_ITEM(args, 0);
    PyObject* pout       = PyTuple_GET_ITEM(args, 1);
    double smoothness   = PyFloat_AsDouble(PyTuple_GET_ITEM(args, 2));
    Input* const input   = reinterpret_cast<Input*>(PyCapsule_GetPointer(pin, "SSDSInput"));
	Output* const output = reinterpret_cast<Output*>(PyCapsule_GetPointer(pout, "SSDSOutput"));
    SkinWeightUpdator weightUpdater(output, input, output->numIndices, smoothness);
    tbb::blocked_range<int> blockedRange(0, input->numVertices);
	tbb::parallel_for(blockedRange, weightUpdater);
    return Py_None;
}
#pragma endregion 

#pragma region Clustering
// see https://sites.google.com/view/fumiyanarita/project/la_ssdr_mdmc
static void detectNeighborClusters(const Input& input, Output& output)
{
    MGlobal::displayInfo("Detecting neighbor clusters");
    const long& numIndices = output.numIndices;

    std::vector<std::set<int>> clusters(output.numJoints);
    for (int v = 0; v < input.numVertices; ++v)
    {
        const int primjid = output.skinIndex[v * numIndices];
        if (input.numRings == 0)
        {
            for (int j = 0; j < output.numJoints; ++j)
            {
                for (int v = 0; v < output.numJoints; ++v)
                {
                    clusters[j].insert(v);
                }
            }
        }
        else
        {
            clusters[primjid].insert(0);
            clusters[primjid].insert(primjid);
            // one-ring neighbors
            for (auto nvit = input.neighbor[v].begin(); nvit != input.neighbor[v].end(); ++nvit)
            {
                const int nvid = nvit->first;
                clusters[primjid].insert(output.skinIndex[nvid * numIndices]);
                if (input.numRings == 1)
                {
                    continue;
                }
                // two-ring neighbors
                for (auto nnvit = input.neighbor[nvid].begin(); nnvit != input.neighbor[nvid].end(); ++nnvit)
                {
                    const int nnvid = nnvit->first;
                    clusters[primjid].insert(output.skinIndex[nnvid * numIndices]);
                }
            }
        }
    }
    for (int v = 0; v < input.numVertices; ++v)
    {
        const int primjid = output.skinIndex[v * numIndices];
        output.vertCluster[v] = std::vector<int>(clusters[primjid].begin(), clusters[primjid].end());
    }
}

long bindVertexToJoint(Output& output, const Input& input)
{
    const long numVertices = input.numVertices;
    const long numSamples  = input.numSamples;
    const long numIndices  = output.numIndices;

    std::vector<int> numJointVertices(output.numJoints, 0);
    for (long v = 0; v < numVertices; ++v)
    {
        long bestJoint = 0;
        double minErr = std::numeric_limits<double>::max();
        const MPoint& restShapePos = input.restShape[v];
        for (long j = 0; j < output.numJoints; ++j)
        {
            double errsq = 0;
            for (long s = 0; s < numSamples; ++s)
            {
                MMatrix am = output.skinMatrix[s * input.numJoints + j].asMatrix();
                MVector diff = input.sample[s * numVertices + v] - restShapePos * am;
                errsq += diff * diff;
            }
            errsq *= (input.restShape[v] - output.restJointPos[j]).length();
            if (errsq < minErr)
            {
                bestJoint = j;
                minErr = errsq;
            }
        }
        ++numJointVertices[bestJoint];
        output.skinIndex[v * numIndices + 0] = bestJoint;
    }

    std::vector<int>::iterator smallestBoneSize = std::min_element(numJointVertices.begin(), numJointVertices.end());
    while (*smallestBoneSize <= 0)
    {
        const long smallestBone = static_cast<int>(smallestBoneSize - numJointVertices.begin());
        numJointVertices.erase(numJointVertices.begin() + smallestBone);
        output.restJointPos.erase(output.restJointPos.begin() + smallestBone);
        for (long s = 0; s < input.numSamples; ++s)
        {
            for (int j = output.numJoints - 2; j >= smallestBone; --j)
            {
                output.skinMatrix[s * input.numJoints + j] = output.skinMatrix[s * input.numJoints + j + 1];
            }
        }
        for (long v = 0; v < numVertices; ++v)
        {
            if (output.skinIndex[v * numIndices + 0] >= smallestBone)
            {
                --output.skinIndex[v * numIndices + 0];
            }
        }
        --output.numJoints;
        smallestBoneSize = std::min_element(numJointVertices.begin(), numJointVertices.end());
    }
    return static_cast<int>(numJointVertices.size());
}

long findMostStableVertex(const Input& input)
{
    const long numVertices = input.numVertices;
    const long numSamples = input.numSamples;
    double minErrorSq = std::numeric_limits<double>::max();
    long mostStableVertex = -1;
    for (int v = 0; v < numVertices; ++v)
    {
        double errSq = 0;
        for (int s = 0; s < numSamples; ++s)
        {
            MVector diff = input.sample[s * numVertices + v] - input.restShape[v];
            errSq += diff * diff;
        }
        if (errSq < minErrorSq)
        {
            minErrorSq = errSq;
            mostStableVertex = v;
        }
    }
    return mostStableVertex;
}

long findDistantVertex(const Input& input, const Output& output, const std::set<long>& covered)
{
    const long numVertices = input.numVertices;
    const long numSamples  = input.numSamples;
    const long numIndices  = output.numIndices;
    double maxErrorSq      = -std::numeric_limits<double>::max();
    long mostDistantVertex = -1;
    for (int v = 0; v < numVertices; ++v)
    {
        if (covered.find(v) != covered.end())
        {
            continue;
        }
        long index = output.skinIndex[v * numIndices + 0];
        double errSq = 0;
        for (int s = 0; s < numSamples; ++s)
        {
            MMatrix bm = output.skinMatrix[s * input.numJoints + index].asMatrix();
            MVector diff = input.sample[s * numVertices + v] - input.restShape[v] * bm;
            errSq += diff * diff;
        }
        if (errSq > maxErrorSq)
        {
            maxErrorSq = errSq;
            mostDistantVertex = v;
        }
    }
    return mostDistantVertex;
}

// solving p-center problem
static PyObject* clusterVerticesPcenter(PyObject* self, PyObject* args)
{
    PyObject* pin = PyTuple_GET_ITEM(args, 0);
    PyObject* pout = PyTuple_GET_ITEM(args, 1);
    const long transformType = PyInt_AsLong(PyTuple_GET_ITEM(args, 2));
    const Input& input = *reinterpret_cast<Input*>(PyCapsule_GetPointer(pin, "SSDSInput"));
    Output& output = *reinterpret_cast<Output*>(PyCapsule_GetPointer(pout, "SSDSOutput"));
    const long& numIndices = output.numIndices;

    output.numJoints = input.numJoints;
    std::fill(output.skinIndex.begin(), output.skinIndex.end(), -1);
    std::fill(output.skinWeight.begin(), output.skinWeight.end(), 0.0);
    std::fill(output.skinMatrix.begin(), output.skinMatrix.end(), MTransformationMatrix::identity);
    std::fill(output.restJointPos.begin(), output.restJointPos.end(), MPoint::origin);

    char buf[512];
    long stableVertex = findMostStableVertex(input);
    output.skinIndex[stableVertex * numIndices] = 0;
    output.skinWeight[stableVertex * numIndices] = 1.0;
    output.restJointPos[0] = input.restShape[stableVertex];
    std::vector<long> jointVertices(output.numJoints);
    jointVertices[0] = stableVertex;
    sprintf(buf, "Added to Vertex %d (stable)", stableVertex);
    MGlobal::displayInfo(buf);

    for (int jid = 1; jid < input.numJoints; ++jid)
    {
        double maxDist = -1.0;
        int distantVertex = -1;
        for (int i = 0; i < input.numVertices; ++i)
        {
            if (std::find(jointVertices.begin(), jointVertices.end(), i) != jointVertices.end())
            {
                continue;
            }
            double mindist = std::numeric_limits<double>::max();
            for (int j = 0; j < jid; ++j)
            {
                const int joint = jointVertices[j];
                MVector v = input.restShape[i] - input.restShape[joint];
                if (mindist > v.length())
                {
                    mindist = v.length();
                }
            }
            if (mindist > maxDist)
            {
                maxDist = mindist;
                distantVertex = i;
            }
        }
        output.restJointPos[jid] = input.restShape[distantVertex];
        jointVertices[jid] = distantVertex;
        sprintf(buf, "Added to Vertex %d, %f", distantVertex, maxDist);
        MGlobal::displayInfo(buf);
    }

    for (int i = 0; i < input.numVertices; ++i)
    {
        double minDist = std::numeric_limits<double>::max();
        int nearestJoint = -1;
        for (int j = 0; j < output.numJoints; ++j)
        {
            const int joint = jointVertices[j];
            MVector v = input.restShape[i] - input.restShape[joint];
            if (v.length() < minDist)
            {
                minDist = v.length();
                nearestJoint = j;
            }
        }
        output.skinIndex[i * numIndices] = nearestJoint;
        output.skinWeight[i * numIndices] = 1.0;
    }
    detectNeighborClusters(input, output);
    updateJointTransformProc(output, transformType, input);
    return PyInt_FromLong(output.numJoints);
}

static PyObject* clusterVerticesAdaptive(PyObject *self, PyObject *args)
{
    PyObject* pin            = PyTuple_GET_ITEM(args, 0);
    PyObject* pout           = PyTuple_GET_ITEM(args, 1);
    const long transformType = PyInt_AsLong(PyTuple_GET_ITEM(args, 2));
    const Input& input = *reinterpret_cast<Input*>(PyCapsule_GetPointer(pin, "SSDSInput"));
    Output& output     = *reinterpret_cast<Output*>(PyCapsule_GetPointer(pout, "SSDSOutput"));
    const long& numIndices = output.numIndices;

    output.numJoints   = 0;
    std::fill(output.skinIndex.begin(), output.skinIndex.end(), -1);
    std::fill(output.skinWeight.begin(), output.skinWeight.end(), 0.0);
    std::fill(output.skinMatrix.begin(), output.skinMatrix.end(), MTransformationMatrix::identity);
    output.restJointPos.clear();

    std::set<long> covered;
    char buf[512];

    long stableVertex = findMostStableVertex(input);
    output.skinIndex[stableVertex * numIndices] = 0;
    output.skinWeight[stableVertex * numIndices] = 1.0;
    output.restJointPos.push_back(input.restShape[stableVertex]);
    covered.insert(stableVertex);
    sprintf(buf, "Added to Vertex %d (stable)", stableVertex);
    MGlobal::displayInfo(buf);

    for (int i = 0; i < input.neighbor[stableVertex].size(); ++i)
    {
        long neighbor = input.neighbor[stableVertex][i].first;
        if (neighbor < 0)
        {
            continue;
        }
        output.skinIndex[neighbor * numIndices] = 0;
        output.skinWeight[neighbor * numIndices] = 1.0;
        covered.insert(neighbor);
    }
    updateJointTransformProc(output, transformType, input, 0);
    for (int v = 0; v < input.numVertices; ++v)
    {
        output.skinIndex[v * numIndices] = 0;
        output.skinWeight[v * numIndices] = 1.0;
    }
    output.numJoints = 1;

    for (int iteration = 0; iteration < input.numJoints + 10; ++iteration)
    {
        if (output.numJoints >= input.numJoints || covered.size() >= input.numVertices)
        {
            break;
        }
        long distantVertex = findDistantVertex(input, output, covered);
        output.skinIndex[distantVertex * numIndices] = output.numJoints;
        output.restJointPos.push_back(input.restShape[distantVertex]);
        covered.insert(distantVertex);
        for (int i = 0; i < input.neighbor[distantVertex].size(); ++i)
        {
            long neighbor = input.neighbor[distantVertex][i].first;
            if (neighbor < 0)
            {
                continue;
            }
            output.skinIndex[neighbor * numIndices] = output.numJoints;
            covered.insert(neighbor);
        }
        updateJointTransformProc(output, transformType, input, output.numJoints);
        ++output.numJoints;
        output.numJoints = bindVertexToJoint(output, input);

        sprintf(buf, "Added to Vertex %d", distantVertex);
        MGlobal::displayInfo(buf);
    }
    detectNeighborClusters(input, output);
    return PyInt_FromLong(output.numJoints);
}
#pragma endregion

#pragma region Initialization
static PyObject* initialize(PyObject *self, PyObject *args)
{
    PyArrayObject* initPos          = reinterpret_cast<PyArrayObject*>(PyTuple_GET_ITEM(args, 0));
    PyArrayObject* shapeSample      = reinterpret_cast<PyArrayObject*>(PyTuple_GET_ITEM(args, 1));
    PyArrayObject* neighborVertices = reinterpret_cast<PyArrayObject*>(PyTuple_GET_ITEM(args, 2));
    const long numJoints            = PyInt_AsLong(PyTuple_GET_ITEM(args, 3));
    const long numIndices           = PyInt_AsLong(PyTuple_GET_ITEM(args, 4));
    const long numRings             = PyInt_AsLong(PyTuple_GET_ITEM(args, 5));
    const long numVertices = static_cast<long>(initPos->dimensions[0]);
    const long numSamples  = static_cast<long>(shapeSample->dimensions[0]);
    // allocation
    Input* input       = new Input();
    input->numVertices = numVertices;
    input->numSamples  = numSamples;
    input->numJoints   = numJoints;
    input->numRings    = numRings;
    input->restShape   = std::vector<MPoint>(numVertices);
    input->sample      = std::vector<MPoint>(numSamples * numVertices);
    input->neighbor    = std::vector<std::vector<std::pair<int, double>>>(numVertices);
    Output* output       = new Output();
    output->numJoints    = 0;
    output->numIndices   = numIndices;
    output->skinIndex    = std::vector<long>(numVertices * numIndices, -1);
    output->skinWeight   = std::vector<double>(numVertices * numIndices, 0.0);
    output->skinMatrix   = std::vector<MTransformationMatrix>(numSamples * numJoints, MTransformationMatrix::identity);
    output->restJointPos = std::vector<MPoint>(numJoints);
    output->vertCluster  = std::vector<std::vector<int>>(numVertices);
    // size normalization
    MVector centroid(0, 0, 0);
    MVector minPos(1.0e10, 1.0e10, 1.0e10), maxPos(-1.0e10, -1.0e10, -1.0e10);
    for (int v = 0; v < numVertices; ++v)
    {
        MVector p(GET_DOUBLE2(initPos, v, 0),
                  GET_DOUBLE2(initPos, v, 1),
                  GET_DOUBLE2(initPos, v, 2));
        input->restShape[v] = p;
        centroid += p;
        for (int i = 0; i < 3; ++i)
        {
            minPos[i] = std::min(minPos[i], p[i]);
            maxPos[i] = std::max(maxPos[i], p[i]);
        }
    }
    centroid /= numVertices;
    double scale = std::max(std::max(maxPos.x - minPos.x, maxPos.y - minPos.y), maxPos.z - minPos.z);
    MMatrix m = MMatrix::identity;
    m(0, 0) = m(1, 1) = m(2, 2) = 1.0 / scale;
    m(3, 0) = -centroid.x / scale;
    m(3, 1) = -centroid.y / scale;
    m(3, 2) = -centroid.z / scale;
    input->normalizer = m;
    // normalized samples
    for (int v = 0; v < numVertices; ++v)
    {
        input->restShape[v] = input->restShape[v] * input->normalizer;
        for (int s = 0; s < numSamples; ++s)
        {
            MPoint p(GET_DOUBLE3(shapeSample, s, v, 0),
                     GET_DOUBLE3(shapeSample, s, v, 1),
                     GET_DOUBLE3(shapeSample, s, v, 2));
            input->sample[s * numVertices + v] = p * input->normalizer;
        }
    }
    // one-ring neighbor [Le and Deng 2014]
    for (int v = 0; v < numVertices; ++v)
    {
		double lv = 0.0;
        for (int i = 0; i < neighborVertices->dimensions[1]; ++i)
        {
            long n = GET_LONG2(neighborVertices, v, i);
            if (n < 0)
            {
                continue;
            }
            double len = (input->restShape[v] - input->restShape[n]).length();
            if (len > 0)
            {
				// [Le and Deng 2014]
				double dsqsum = 0;
				for (int s = 0; s < input->numSamples; ++s)
				{
					float diff = (input->restShape[v] - input->restShape[n]).length()
						- (input->sample[s * numVertices + v] - input->sample[s * numVertices + n]).length();
					dsqsum += diff * diff;
				}
				dsqsum += 1.0e-10;
				double dvn = 1.0 / std::sqrt(dsqsum / input->numSamples);
                input->neighbor[v].push_back(std::make_pair(n, dvn));
                lv += dvn;
            }
        }
		for (int n = 0; n < input->neighbor[v].size(); ++n)
		{
			input->neighbor[v][n].second /= -lv;
		}
	}
    PyObject* retval = PyTuple_New(2);
    PyTuple_SetItem(retval, 0, PyCapsule_New(input,  "SSDSInput", nullptr));
    PyTuple_SetItem(retval, 1, PyCapsule_New(output, "SSDSOutput", nullptr));
    return retval;
}

static PyObject* release(PyObject *self, PyObject *args)
{
	PyObject* pin = PyTuple_GET_ITEM(args, 0);
	PyObject* pout = PyTuple_GET_ITEM(args, 1);
	delete reinterpret_cast<Input*>(PyCapsule_GetPointer(pin, "SSDSInput"));
	delete reinterpret_cast<Output*>(PyCapsule_GetPointer(pout, "SSDSOutput"));
	return Py_None;
}

static PyObject* retrieveResult(PyObject *self, PyObject *args)
{
	PyObject* pin                = PyTuple_GET_ITEM(args, 0);
	PyObject* pout               = PyTuple_GET_ITEM(args, 1);
	PyArrayObject* skinIndex     = reinterpret_cast<PyArrayObject*>(PyTuple_GET_ITEM(args, 2));
	PyArrayObject* skinWeight    = reinterpret_cast<PyArrayObject*>(PyTuple_GET_ITEM(args, 3));
	PyArrayObject* skinMatrix    = reinterpret_cast<PyArrayObject*>(PyTuple_GET_ITEM(args, 4));
    PyArrayObject* restJointPos = reinterpret_cast<PyArrayObject*>(PyTuple_GET_ITEM(args, 5));
    Input* const input     = reinterpret_cast<Input*>(PyCapsule_GetPointer(pin, "SSDSInput"));
	Output* const output   = reinterpret_cast<Output*>(PyCapsule_GetPointer(pout, "SSDSOutput"));
    const long& numIndices = output->numIndices;

    MMatrix inm = input->normalizer.inverse();
	for (int s = 0; s < input->numSamples; ++s)
	{
		for (int b = 0; b < output->numJoints; ++b)
		{
			MMatrix m = input->normalizer * output->skinMatrix[s * output->numJoints + b].asMatrix() * inm;
			for (int i = 0; i < 4; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					SET_DOUBLE4(skinMatrix, b, s, i, j, m(i, j));
				}
			}
        }
	}
    for (int v = 0; v < input->numVertices; ++v)
    {
        for (long i = 0; i < numIndices; ++i)
        {
            SET_LONG2(skinIndex, v, i, output->skinIndex[v * numIndices + i]);
            SET_DOUBLE2(skinWeight, v, i, output->skinWeight[v * numIndices + i]);
        }
    }
    for (int j = 0; j < output->numJoints; ++j)
    {
        MPoint cp = output->restJointPos[j] * inm;
        SET_DOUBLE2(restJointPos, j, 0, cp.x);
        SET_DOUBLE2(restJointPos, j, 1, cp.y);
        SET_DOUBLE2(restJointPos, j, 2, cp.z);
    }
	return Py_None;
}
#pragma endregion

static PyMethodDef methods[] = {
    { "initialize", initialize, METH_VARARGS, "initialize" },
    { "clusterVerticesPcenter", clusterVerticesPcenter, METH_VARARGS, "clusterVerticesPcenter" },
    { "clusterVerticesAdaptive", clusterVerticesAdaptive, METH_VARARGS, "clusterVerticesAdaptive" },
    { "updateJointTransform", updateJointTransform, METH_VARARGS, "updateJointTransform" },
    { "updateSkinWeight", updateSkinWeight, METH_VARARGS, "updateSkinWeight" },
	{ "retrieveResult", retrieveResult, METH_VARARGS, "retrieveResult" },
	{ "release", release, METH_VARARGS, "release" },
	{ NULL, NULL, 0, NULL }
};

PyMODINIT_FUNC initnative(void)
{
    Py_InitModule("native", methods);
    import_array();
	Eigen::initParallel();
}
