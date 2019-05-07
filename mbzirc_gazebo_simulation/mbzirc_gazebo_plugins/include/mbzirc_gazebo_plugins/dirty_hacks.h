//
// Original code from: https://pastebin.com/g5E7Bu0z
//
#ifndef DIRTY_HACKS_H
#define DIRTY_HACKS_H

namespace dirty_hacks {
template <typename Tag, typename Tag::type M>
struct Rob {
    friend typename Tag::type get(Tag) {
        return M;
    }
};

template <typename T, typename C, typename M>
struct tag_base {
    using type = M C::*;
    friend type get(T);
};
} // namespace dirty_hacks

#define ENABLE_ACCESS(tag_name, class_name, member_name, member_type)     \
    namespace dirty_hacks {                                               \
        struct tag_name : tag_base<tag_name, class_name, member_type> {}; \
        template struct Rob<tag_name, &class_name::member_name>;          \
    } // namespace dirty_hacks

#define ACCESS(object, tag_name) ((object).*get(::dirty_hacks::tag_name()))

#endif // DIRTY_HACKS_H