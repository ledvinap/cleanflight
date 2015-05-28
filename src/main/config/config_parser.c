
// .config.version
// .config.features.BLACKBOX
// .config.escandservo.minthrottle
// .config.customMixer.1.throttle
// .config.profile.1.gpsProfile.gps_lpf
// .config.profile.1.adjustment.1.range.start
// .config.profile.1.mode.id

#include <stdbool.h>
#include <stdio.h>

#include "config/config_parser.h"
#include "config/config_def.h"


struct cdef_metainfo_expanded {
    enum cdef_metainfo_type metainfo_type;
    int datOffset;        // data offset in containing entity
    int datSize;          // data size of this field
    const union cdef_metainfo_typedef *subDef;   // pointer to first child
    const union cdef_metainfo_typedef *nextDef;  // pointer to next sibbling/parrent
    bool last;

    const char *name;  // name if field is named

    int arrayStride;
    int arrayCount;

    const void *tagDflt, *tagMin, *tagMax;
    enum cdef_primitiveType primitiveType;
};

#define PTR_ADD(ptr, offset) ((typeof (ptr))((uint8_t*)(ptr) + (offset)))
#define ALIGNOF(type) offsetof (struct { char c; type member; }, member)

// get unsigned bits from type
#define BF_GET(src, ofs, bits) (((src) >> ofs) & ((1 << bits) - 1))
// return bvalue shifted to correct position
#define BF_SET(ofs, bits, val) (((val) & ((1 << bits) - 1)) << bits)

#define POP_FIELD(type, member) ({                      \
            typeof (type->member) _ret = type->member;  \
            type = PTR_ADD(type, sizeof(type->member)); \
            _ret;                                        \
        })
#define POP_PTR(type, size) ({                  \
            const void* _ret = type;                           \
            type = PTR_ADD(type, size); \
            _ret;                                        \
        })

int cdef_type_size(const union cdef_metainfo_typedef *type, int *defSize, int *datSize);
int cdef_primitive_size(enum cdef_primitiveType ptype);

int cdef_expand(struct cdef_metainfo_expanded *dst, const union cdef_metainfo_typedef *type) {
    const union cdef_metainfo_typedef *typeStart = type;
    int flags = POP_FIELD(type, flags);
    dst->metainfo_type = BF_GET(flags, MF_TYPE_OFS, MF_TYPE_SZE);
    dst->last = flags &  MF_LAST;
    int defSize = (flags & MF_DEFSIZE) ? POP_FIELD(type, defSize) : 0;
    int datSize = (flags & MF_DATSIZE) ? POP_FIELD(type, datSize) : 0;
    dst->name = (flags & MF_NAMEPTR) ? POP_FIELD(type, namePtr) : NULL;
    dst->datOffset = (flags & MF_DATAOFFSET) ? POP_FIELD(type, dataOffset) : 0;
    dst->arrayStride = (flags & MF_ARRAY_STRIDE) ? POP_FIELD(type, arrayStride) : 0;
    dst->arrayCount = (flags & MF_ARRAY_COUNT) ? POP_FIELD(type, arrayCount) : 0;

    dst->subDef = NULL;
    int subData = BF_GET(flags, MF_SUBDATA_OFS, MF_SUBDATA_SZE);
    int subDataSize = 0, subDefSize = 0;
    switch(subData) {
    case MF_SUBDATA_REF:
        dst->subDef = POP_FIELD(type, defPtr);
        if(!datSize) {
            // we need to know subdata size
            cdef_type_size(dst->subDef, NULL, &subDataSize);
        }
        break;
    case MF_SUBDATA_DIRECT: // in-place subdata record
        dst->subDef = type;
        cdef_type_size(dst->subDef, &subDefSize, &subDataSize);
        type = PTR_ADD(type, subDefSize);
        break;
    case MF_SUBDATA_PRIMITIVE:
        dst->primitiveType = POP_FIELD(type, primitiveType);
        dst->subDef = NULL;
        if(!datSize) {
            subDataSize = cdef_primitive_size(dst->primitiveType);
        }
    }
    if(!datSize) {
        datSize = (dst->arrayCount > 0 ? dst->arrayCount : 1) * subDataSize;
    }
    dst->datSize = datSize;
    
    dst->tagDflt = (flags & MF_DFLT) ? POP_PTR(type, datSize) : NULL;
    dst->tagMin = (flags & MF_MIN) ? POP_PTR(type, datSize) : NULL;
    dst->tagMax = (flags & MF_MAX) ? POP_PTR(type, datSize) : NULL;

    if(defSize == 0) {
        dst->nextDef = type;
    } else {
        if(type != PTR_ADD(typeStart, defSize)) {
            // TODO!
            printf("defSize mismatch\n");
        }
        dst->nextDef = PTR_ADD(typeStart, defSize);
    }
    return 0;
}

// return size of metainfo and data structure
// TODO - define cases when this may be called - only trivial definitions should need this, nontrivial cases should be specified explicitly
int cdef_type_size(const union cdef_metainfo_typedef *type, int *defSize, int *datSize)
{
    const union cdef_metainfo_typedef *typeStart = type;
    struct cdef_metainfo_expanded expt;
    int datSz;
    do {
        cdef_expand(&expt, type);
        datSz = expt.datOffset + expt.datSize;
        type = expt.nextDef;
    } while(!expt.last);
    if(defSize)
        *defSize = (char*)type - (char*)typeStart;
    if(datSize)
        *datSize = datSz;
    return 0;
}

static uint16_t primitive_size_tab[] = {
    [cdef_primitive_NONE] = 0,
    [cdef_primitive_UINT16] = sizeof(uint16_t),
};

int cdef_primitive_size(enum cdef_primitiveType ptype)
{
    return primitive_size_tab[ptype];
}

int cdef_dump(const union cdef_metainfo_typedef *type, void *data, int depth)
{
#if 0
    const union cdef_metainfo_typedef *type = *typePtr;
    void* data = *dataPtr;
#endif
    struct cdef_metainfo_expanded expt;
    do {
        cdef_expand(&expt, type);
        switch (expt.metainfo_type) {
        case cdef_field:
            printf("%*s.%s:offset=%d,size=%d\n", depth, "", expt.name, expt.datOffset, expt.datSize);
            cdef_dump(expt.subDef, data + expt.datOffset, depth + 1);
            break;
        case cdef_array:
            printf("%*sARRAY[%d],offset=%d,size=%d,stride=%d\n", depth, "", expt.arrayCount, expt.datOffset, expt.datSize, expt.arrayStride);
            for (int i = 0; i < expt.arrayCount; i++)
                cdef_dump(expt.subDef, data + expt.datOffset + i *  expt.arrayStride, depth + 1);
            break;
        case cdef_primitive:
            printf("%*sT(%d):offset=%d,size=%d\n",  depth, "", expt.primitiveType, expt.datOffset, expt.datSize);
            break;
        }
        type = expt.nextDef;
    } while (!expt.last);
    return 0;
}
#if 0
// search for name, tranversing metainfo structure from type and data structure from ptr
// name, type and data are modified as tree is transversed and returned in the end
int cdef_find(const char** name, const union cdef_metainfo_typedef** typePtr, void** dataPtr);

// perform one search step
int cdef_find_step(const char** path_ptr, const char* path_end, const union cdef_metainfo_typedef **typePtr, void **dataPtr)
{
    int ret;
    // save some writing ...
    const char *path = *path_ptr;

    if(!path || path[0] != '.') {
        // end of descent, return current position
        return CDEF_FOUND;
    }

    path++;                                   // skip '.'
    char name_buff[MAX_CDEF_NAME_LEN];
    const char * name;
    const char* eptr = strchr(path, '.');
    if(eptr) {
        // we want to null-terminated string, so make a copy
        memcpy(name_buff, path, eptr-path);
        name = name_buff;
    } else {
        name = path;
    }
    *path_ptr = path = eptr;

    // descend if possible
    switch(type->type) {
    case cdef_struct:
        ret = cdef_find_stuct(name, type_ptr, data_ptr);
        break;
    case cdef_arrray:
        ret = cdef_find_array(name, type_ptr, data_ptr);
        break;
    default:
        return CDEF_INVALID_TYPE;
    }


}

int cdef_find_stuct(const char* name, const cdef_metainfo_header_t* *type_ptr, void* *data_ptr)
{
    cdef_metainfo_header_t *type = *type_ptr;
    struct  cdef_metainfo_data_struct_t *field = &type->data[0].structure;
    while(field < PRT_ADD(type, type->size)) {
        if(strcmp(name, field->member_name) == 0) {
            // OK, found requested member.
            *type_ptr = field->member_data;
            *data_ptr = field->member_offset;
            return CDEF_FOUND;
        }
        (const uint8_t*)field += field->member_data[0].size;
    }
}

int cdef_find_array(const char* name, const cdef_metainfo_header_t* *type_ptr, void* *data_ptr)
{
    cdef_metainfo_header_t *type = *type_ptr;
    int index = atoi(name);  // TODO - check

    int item_count = 0;
    int item_offset = 0;
    int item_stride = 0;

    union cdef_metainfo_data_array_t *hdr = &type->data[0].array;

    uint16_t flags = hdr->hdr.flags;
    hdr = PTR_ADD(hdr, sizeof(hdr->hdr));
    if(flags & CDEF_ARRAY_FLAG_COUNT) {
        item_count = hdr->count;
        hdr = PTR_ADD(hdr, sizeof(hdr->count));
    }
    if(flags & CDEF_ARRAY_FLAG_OFFSET) {
        item_offset = hdr->offset;
        hdr = PTR_ADD(hdr, sizeof(hdr->offset));
    }
    if(flags & CDEF_ARRAY_FLAG_STRIDE) {
        item_stride = hdr->stride;
        hdr = PTR_ADD(hdr, sizeof(hdr->stride));
    }
    *type_ptr = hdr->member_data;
    if(index >= 0 && index < item_count) {
        *data_ptr += index * stride + offset;
        return CDEF_FOUND;
    }
    return CDEF_RANGEERR;
}

int cdef_follow_ref(const cdef_metainfo_header_t* *type_ptr, void* *data_ptr)
{
    cdef_metainfo_header_t *type = *type_ptr;
    cdef_metainfo_data_ref_t *ref = &type->data[0].ref;
    *type_ptr = ref->data;
}
#endif
