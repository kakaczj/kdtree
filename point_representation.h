#pragma once
#include <vector>
#include <algorithm>
#include <memory>
namespace pcl
{
/** \brief @b PointRepresentation provides a set of methods for converting a point structs/object into an
    * n-dimensional vector.
    * \note This is an abstract class.  Subclasses must set nr_dimensions_ to the appropriate value in the constructor
    * and provide an implementation of the pure virtual copyToFloatArray method.
    * \author Michael Dixon
    */
template <typename PointT>
class PointRepresentation{
protected:
    /** \brief The number of dimensions in this point's vector (i.e. the "k" in "k-D") */
    int nr_dimensions_ = 0;
    /** \brief A vector containing the rescale factor to apply to each dimension. */
    std::vector<float> alpha_;
    /** \brief Indicates whether this point representation is trivial. It is trivial if and only if the following
        *  conditions hold:
        *  - the relevant data consists only of float values
        *  - the vectorize operation directly copies the first nr_dimensions_ elements of PointT to the out array
        *  - sizeof(PointT) is a multiple of sizeof(float)
        *  In short, a trivial point representation converts the input point to a float array that is the same as if
        *  the point was reinterpret_casted to a float array of length nr_dimensions_ . This value says that this
        *  representation can be trivial; it is only trivial if setRescaleValues() has not been set.
        */
    bool trivial_ = false;

public:
    using Ptr = std::shared_ptr<PointRepresentation<PointT> >;
    using ConstPtr = std::shared_ptr<const PointRepresentation<PointT> >;

    /** \brief Empty destructor */
    virtual ~PointRepresentation() = default;
    //TODO: check if copy and move constructors / assignment operators are needed

    /** \brief Copy point data from input point to a float array. This method must be overridden in all subclasses.
        *  \param[in] p The input point
        *  \param[out] out A pointer to a float array.
        */
    virtual void copyToFloatArray(const PointT& p, float* out) const = 0;

    /** \brief Returns whether this point representation is trivial. It is trivial if and only if the following
        *  conditions hold:
        *  - the relevant data consists only of float values
        *  - the vectorize operation directly copies the first nr_dimensions_ elements of PointT to the out array
        *  - sizeof(PointT) is a multiple of sizeof(float)
        *  In short, a trivial point representation converts the input point to a float array that is the same as if
        *  the point was reinterpret_casted to a float array of length nr_dimensions_ . */
    inline bool isTrivial() const { return trivial_ && alpha_.empty(); }

    /** \brief Verify that the input point is valid.
        *  \param p The point to validate
        */
    virtual bool
        isValid(const PointT& p) const{
        bool is_valid = true;

        if (trivial_){
            const float* temp = reinterpret_cast<const float*>(&p);

            for (int i = 0; i < nr_dimensions_; ++i){
                if (!std::isfinite(temp[i])){
                    is_valid = false;
                    break;
                }
            }
        }
        else{
            float* temp = new float[nr_dimensions_];
            copyToFloatArray(p, temp);

            for (int i = 0; i < nr_dimensions_; ++i){
                if (!std::isfinite(temp[i])){
                    is_valid = false;
                    break;
                }
            }
            delete[] temp;
        }
        return (is_valid);
    }

    /** \brief Convert input point into a vector representation, rescaling by \a alpha.
        * \param[in] p the input point
        * \param[out] out The output vector.  Can be of any type that implements the [] operator.
        */
    template <typename OutputType> void
        vectorize(const PointT& p, OutputType& out) const{
        float* temp = new float[nr_dimensions_];
        copyToFloatArray(p, temp);
        if (alpha_.empty()){
            for (int i = 0; i < nr_dimensions_; ++i)
                out[i] = temp[i];
        }
        else{
            for (int i = 0; i < nr_dimensions_; ++i)
                out[i] = temp[i] * alpha_[i];
        }
        delete[] temp;
    }

    /** \brief Set the rescale values to use when vectorizing points
        * \param[in] rescale_array The array/vector of rescale values.  Can be of any type that implements the [] operator.
        */
    void
        setRescaleValues(const float* rescale_array){
        alpha_.resize(nr_dimensions_);
        std::copy(rescale_array, rescale_array + nr_dimensions_, alpha_.begin());
    }

    /** \brief Return the number of dimensions in the point's vector representation. */
    inline int getNumberOfDimensions() const { return (nr_dimensions_); }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b DefaultPointRepresentation extends PointRepresentation to define default behavior for common point types.
    */
template <typename PointDefault>
class DefaultPointRepresentation : public PointRepresentation <PointDefault>{
    using PointRepresentation <PointDefault>::nr_dimensions_;
    using PointRepresentation <PointDefault>::trivial_;

public:
    // Boost shared pointers
    using Ptr = std::shared_ptr<DefaultPointRepresentation<PointDefault> >;
    using ConstPtr = std::shared_ptr<const DefaultPointRepresentation<PointDefault> >;

    DefaultPointRepresentation(){
        // If point type is unknown, assume it's a struct/array of floats, and compute the number of dimensions
        nr_dimensions_ = sizeof(PointDefault) / sizeof(float);
        // Limit the default representation to the first 3 elements
        if (nr_dimensions_ > 3) nr_dimensions_ = 3;

        trivial_ = true;
    }

    ~DefaultPointRepresentation() override = default;

    inline Ptr
        makeShared() const{
        return (Ptr(new DefaultPointRepresentation<PointDefault>(*this)));
    }

    void
        copyToFloatArray(const PointDefault& p, float* out) const override{
        // If point type is unknown, treat it as a struct/array of floats
        const float* ptr = reinterpret_cast<const float*> (&p);
        std::copy(ptr, ptr + nr_dimensions_, out);
    }
};
}
