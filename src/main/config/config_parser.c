
// .config.version
// .config.features.BLACKBOX
// .config.escandservo.minthrottle
// .config.customMixer.1.throttle
// .config.profile.1.gpsProfile.gps_lpf
// .config.profile.1.adjustment.1.range.start
// .config.profile.1.mode.id

#define PTR_ADD(ptr, offset) ((typeof ptr)((uint8_t*)ptr + offset))

// search for name, tranversing metainfo structure from type and data structure from ptr
// name, type and data are modified as tree is transversed and returned in the end
int cdef_find(const char** name, const cdef_metainfo_header_t** type, void** data);

// perform one search step
int cdef_find_step(const char** path_ptr, const cdef_metainfo_header_t** type_ptr, void** data_ptr)
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
        name = name|_buff;
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
    while(field < (const uint8_t*)type + type->size) {
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
